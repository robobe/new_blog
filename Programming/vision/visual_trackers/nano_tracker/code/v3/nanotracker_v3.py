import cv2
import numpy as np
import onnxruntime as ort


# ------------------------------------------------------------
# NanoTrack V3 configuration
# ------------------------------------------------------------

EXEMPLAR_SIZE = 127
INSTANCE_SIZE = 255

STRIDE = 16
CONTEXT_AMOUNT = 0.5

WINDOW_INFLUENCE = 0.455
PENALTY_K = 0.138
LR = 0.348


# ------------------------------------------------------------
# Utility functions
# ------------------------------------------------------------

def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def change(r):
    """
    max(r, 1/r)
    """
    return np.maximum(r, 1.0 / (r + 1e-12))


def sz(w, h):
    """
    Size including padding.

    Equivalent to NanoTrack's sz_whFun / sz_change logic.
    """
    pad = (w + h) * 0.5
    return np.sqrt((w + pad) * (h + pad))


def get_subwindow(
    image,
    center,
    model_size,
    original_size,
    avg_chans,
):
    """
    Crop a square region centered on `center`.

    If the region goes outside the image, pad it using
    the average image color.

    Then resize to model_size x model_size.
    """

    original_size = int(round(original_size))

    cx, cy = center

    # Same idea as the reference NanoTrack implementation.
    c = (original_size + 1) / 2.0

    xmin = int(round(cx - c))
    ymin = int(round(cy - c))

    xmax = xmin + original_size - 1
    ymax = ymin + original_size - 1

    left_pad = max(0, -xmin)
    top_pad = max(0, -ymin)
    right_pad = max(0, xmax - image.shape[1] + 1)
    bottom_pad = max(0, ymax - image.shape[0] + 1)

    xmin += left_pad
    xmax += left_pad
    ymin += top_pad
    ymax += top_pad

    if left_pad or top_pad or right_pad or bottom_pad:
        padded = cv2.copyMakeBorder(
            image,
            top_pad,
            bottom_pad,
            left_pad,
            right_pad,
            cv2.BORDER_CONSTANT,
            value=tuple(float(v) for v in avg_chans),
        )
    else:
        padded = image

    crop = padded[
        ymin:ymax + 1,
        xmin:xmax + 1
    ]

    if crop.shape[0] != model_size or crop.shape[1] != model_size:
        crop = cv2.resize(
            crop,
            (model_size, model_size),
            interpolation=cv2.INTER_LINEAR,
        )

    return crop


def prepare_backbone_input(image):
    """
    OpenCV gives BGR.
    NanoTrack backbone expects RGB.

    No ImageNet mean/std normalization is used by
    the NanoTrack deployment reference implementation.
    """

    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # HWC -> CHW
    tensor = np.transpose(rgb, (2, 0, 1))

    # add batch dimension
    tensor = tensor[np.newaxis, ...]

    return np.ascontiguousarray(tensor.astype(np.float32))


# ------------------------------------------------------------
# ONNX helpers
# ------------------------------------------------------------

class Backbone:
    def __init__(self, model_path):
        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"],
        )

        self.input = self.session.get_inputs()[0]
        self.output = self.session.get_outputs()[0]

        print("Backbone")
        print(
            f"  input : {self.input.name} "
            f"{self.input.shape} {self.input.type}"
        )
        print(
            f"  output: {self.output.name} "
            f"{self.output.shape} {self.output.type}"
        )

    def __call__(self, image):
        x = prepare_backbone_input(image)

        result = self.session.run(
            None,
            {
                self.input.name: x,
            },
        )

        return result[0]


class Head:
    def __init__(self, model_path):
        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"],
        )

        self.inputs = self.session.get_inputs()
        self.outputs = self.session.get_outputs()

        print("\nHead")

        for inp in self.inputs:
            print(
                f"  input : {inp.name} "
                f"{inp.shape} {inp.type}"
            )

        for out in self.outputs:
            print(
                f"  output: {out.name} "
                f"{out.shape} {out.type}"
            )

    def __call__(self, zf, xf):

        if len(self.inputs) != 2:
            raise RuntimeError(
                f"Expected head to have 2 inputs, "
                f"found {len(self.inputs)}"
            )

        result = self.session.run(
            None,
            {
                self.inputs[0].name: zf,
                self.inputs[1].name: xf,
            },
        )

        if len(result) != 2:
            raise RuntimeError(
                f"Expected head to return 2 tensors, "
                f"found {len(result)}"
            )

        #
        # One output should be classification:
        #     [1, 2, H, W]
        #
        # One output should be bbox:
        #     [1, 4, H, W]
        #
        cls = None
        bbox = None

        for output in result:
            print_shape = output.shape

            if output.ndim != 4:
                continue

            if output.shape[1] == 2:
                cls = output

            elif output.shape[1] == 4:
                bbox = output

        if cls is None or bbox is None:
            print("Head result shapes:")
            for result_tensor in result:
                print(" ", result_tensor.shape)

            raise RuntimeError(
                "Could not identify classification/bbox outputs"
            )

        return cls, bbox


# ------------------------------------------------------------
# NanoTrack
# ------------------------------------------------------------

class NanoTrack:
    def __init__(self, backbone_path, head_path):

        self.backbone = Backbone(backbone_path)
        self.head = Head(head_path)

        self.center = None
        self.target_size = None
        self.avg_chans = None

        self.image_width = None
        self.image_height = None

        self.zf = None

        self.window = None
        self.grid_x = None
        self.grid_y = None

    # --------------------------------------------------------

    def init(self, image, bbox):
        """
        bbox:
            (x, y, width, height)
        """

        x, y, w, h = bbox

        self.center = np.array(
            [
                x + w / 2.0,
                y + h / 2.0,
            ],
            dtype=np.float32,
        )

        self.target_size = np.array(
            [w, h],
            dtype=np.float32,
        )

        self.image_height, self.image_width = image.shape[:2]

        #
        # The reference implementation stores the mean
        # channel value for padding.
        #
        self.avg_chans = image.mean(axis=(0, 1))

        # context around target
        wc_z = (
            w
            + CONTEXT_AMOUNT * (w + h)
        )

        hc_z = (
            h
            + CONTEXT_AMOUNT * (w + h)
        )

        s_z = round(np.sqrt(wc_z * hc_z))

        #
        # Template crop: 127x127
        #
        z_crop = get_subwindow(
            image,
            self.center,
            EXEMPLAR_SIZE,
            s_z,
            self.avg_chans,
        )

        #
        # Backbone is run once for the template.
        #
        self.zf = self.backbone(z_crop)

        print("\nTemplate feature:", self.zf.shape)

    # --------------------------------------------------------

    def _create_window_and_grid(self, score_size):

        hanning = np.hanning(score_size)

        self.window = np.outer(
            hanning,
            hanning,
        ).astype(np.float32)

        #
        # Every score-map location corresponds to a
        # position in the 255x255 search image.
        #
        coordinate = np.arange(
            score_size,
            dtype=np.float32,
        ) * STRIDE

        self.grid_x, self.grid_y = np.meshgrid(
            coordinate,
            coordinate,
        )

    # --------------------------------------------------------

    def track(self, image):

        target_w, target_h = self.target_size

        #
        # Calculate search-region size.
        #
        wc_z = (
            target_w
            + CONTEXT_AMOUNT
            * (target_w + target_h)
        )

        hc_z = (
            target_h
            + CONTEXT_AMOUNT
            * (target_w + target_h)
        )

        s_z = np.sqrt(wc_z * hc_z)

        scale_z = EXEMPLAR_SIZE / s_z

        d_search = (
            INSTANCE_SIZE - EXEMPLAR_SIZE
        ) / 2.0

        pad = d_search / scale_z

        s_x = s_z + 2.0 * pad

        #
        # Search crop: 255x255
        #
        x_crop = get_subwindow(
            image,
            self.center,
            INSTANCE_SIZE,
            s_x,
            self.avg_chans,
        )

        #
        # Search backbone inference.
        #
        xf = self.backbone(x_crop)

        #
        # Siamese head.
        #
        cls, bbox = self.head(
            self.zf,
            xf,
        )

        #
        # Expected:
        #
        # cls:
        #    [1, 2, H, W]
        #
        # bbox:
        #    [1, 4, H, W]
        #
        score_size = cls.shape[2]

        if self.window is None:
            self._create_window_and_grid(
                score_size
            )

            print(
                "Score map:",
                score_size,
                "x",
                score_size,
            )

        # ----------------------------------------------------
        # Classification
        # ----------------------------------------------------

        #
        # Reference NanoTrack uses channel 1 as foreground.
        #
        cls_score = cls[0, 1]

        score = sigmoid(cls_score)

        # ----------------------------------------------------
        # Bounding-box regression
        # ----------------------------------------------------

        #
        # bbox channels represent:
        #
        #   left
        #   top
        #   right
        #   bottom
        #
        left = bbox[0, 0]
        top = bbox[0, 1]
        right = bbox[0, 2]
        bottom = bbox[0, 3]

        pred_x1 = self.grid_x - left
        pred_y1 = self.grid_y - top

        pred_x2 = self.grid_x + right
        pred_y2 = self.grid_y + bottom

        pred_w = pred_x2 - pred_x1
        pred_h = pred_y2 - pred_y1

        # ----------------------------------------------------
        # Scale / ratio penalty
        # ----------------------------------------------------

        #
        # Target size must first be expressed in
        # search-image coordinates.
        #
        scaled_target_w = (
            self.target_size[0] * scale_z
        )

        scaled_target_h = (
            self.target_size[1] * scale_z
        )

        target_sz = sz(
            scaled_target_w,
            scaled_target_h,
        )

        prediction_sz = sz(
            pred_w,
            pred_h,
        )

        s_c = change(
            prediction_sz / target_sz
        )

        target_ratio = (
            scaled_target_w
            / scaled_target_h
        )

        prediction_ratio = (
            pred_w
            / (pred_h + 1e-12)
        )

        r_c = change(
            target_ratio
            / prediction_ratio
        )

        penalty = np.exp(
            -(s_c * r_c - 1.0)
            * PENALTY_K
        )

        # ----------------------------------------------------
        # Hanning window penalty
        # ----------------------------------------------------

        pscore = (
            penalty
            * score
            * (1.0 - WINDOW_INFLUENCE)
            + self.window
            * WINDOW_INFLUENCE
        )

        #
        # Highest scoring location.
        #
        best_index = np.unravel_index(
            np.argmax(pscore),
            pscore.shape,
        )

        row, col = best_index

        best_score = float(
            score[row, col]
        )

        # ----------------------------------------------------
        # Decode bbox at best cell
        # ----------------------------------------------------

        x1 = pred_x1[row, col]
        y1 = pred_y1[row, col]

        x2 = pred_x2[row, col]
        y2 = pred_y2[row, col]

        pred_center_x = (
            x1 + x2
        ) / 2.0

        pred_center_y = (
            y1 + y2
        ) / 2.0

        pred_width = x2 - x1
        pred_height = y2 - y1

        #
        # Convert displacement relative to search center.
        #
        diff_x = (
            pred_center_x
            - INSTANCE_SIZE / 2.0
        )

        diff_y = (
            pred_center_y
            - INSTANCE_SIZE / 2.0
        )

        #
        # Search coordinates -> original image coordinates.
        #
        diff_x /= scale_z
        diff_y /= scale_z

        pred_width /= scale_z
        pred_height /= scale_z

        # ----------------------------------------------------
        # Update target state
        # ----------------------------------------------------

        lr = (
            penalty[row, col]
            * score[row, col]
            * LR
        )

        self.center[0] += diff_x
        self.center[1] += diff_y

        self.target_size[0] = (
            pred_width * lr
            + self.target_size[0]
            * (1.0 - lr)
        )

        self.target_size[1] = (
            pred_height * lr
            + self.target_size[1]
            * (1.0 - lr)
        )

        # ----------------------------------------------------
        # Clip target
        # ----------------------------------------------------

        self.center[0] = np.clip(
            self.center[0],
            0,
            self.image_width,
        )

        self.center[1] = np.clip(
            self.center[1],
            0,
            self.image_height,
        )

        self.target_size[0] = np.clip(
            self.target_size[0],
            10,
            self.image_width,
        )

        self.target_size[1] = np.clip(
            self.target_size[1],
            10,
            self.image_height,
        )

        # ----------------------------------------------------
        # Return x, y, w, h
        # ----------------------------------------------------

        x = (
            self.center[0]
            - self.target_size[0] / 2.0
        )

        y = (
            self.center[1]
            - self.target_size[1] / 2.0
        )

        return (
            float(x),
            float(y),
            float(self.target_size[0]),
            float(self.target_size[1]),
            best_score,
        )


# ------------------------------------------------------------
# Application
# ------------------------------------------------------------

def main():

    backbone_path = (
        "models/nanotrackv3/"
        "nanotrack_backbone.onnx"
    )

    head_path = (
        "models/nanotrackv3/"
        "nanotrack_head.onnx"
    )

    tracker = NanoTrack(
        backbone_path,
        head_path,
    )

    cap = cv2.VideoCapture("video.mp4")

    if not cap.isOpened():
        raise RuntimeError(
            "Could not open video"
        )

    ok, frame = cap.read()

    if not ok:
        raise RuntimeError(
            "Could not read first frame"
        )

    #
    # Select the object manually.
    #
    bbox = cv2.selectROI(
        "NanoTrack V3",
        frame,
        fromCenter=False,
        showCrosshair=True,
    )

    cv2.destroyWindow(
        "NanoTrack V3"
    )

    if bbox[2] <= 0 or bbox[3] <= 0:
        raise RuntimeError(
            "Invalid ROI"
        )

    tracker.init(
        frame,
        bbox,
    )

    while True:

        ok, frame = cap.read()

        if not ok:
            break

        x, y, w, h, score = tracker.track(
            frame
        )

        x1 = int(x)
        y1 = int(y)

        x2 = int(x + w)
        y2 = int(y + h)

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        cv2.putText(
            frame,
            f"score={score:.3f}",
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        cv2.imshow(
            "NanoTrack V3",
            frame,
        )

        key = cv2.waitKey(1) & 0xFF

        if key == 27 or key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()