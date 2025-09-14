#!/usr/bin/env python3

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import threading
import time

class GStreamerPipelineWithRemoval:
    def __init__(self):
        Gst.init(None)
        self.pipeline = None
        self.loop = None
        self.right_source_removed = False
        
    def create_pipeline_from_string(self):
        """Create pipeline from string description"""
        # Pipeline string for side-by-side compositor
        pipeline_string = """
        compositor name=comp 
            sink_0::xpos=0 sink_0::ypos=0 sink_0::width=320 sink_0::height=240 
            sink_1::xpos=320 sink_1::ypos=0 sink_1::width=320 sink_1::height=240 ! 
        video/x-raw,width=640,height=240 ! 
        videoconvert ! 
        autovideosink 
        videotestsrc name=leftsrc pattern=0 ! 
            video/x-raw,width=320,height=240 ! 
            comp.sink_0 
        videotestsrc name=rightsrc pattern=1 ! 
            video/x-raw,width=320,height=240 ! 
            comp.sink_1
        """.replace('\n', ' ').strip()
        
        try:
            print(f"Creating pipeline: {pipeline_string}")
            self.pipeline = Gst.parse_launch(pipeline_string)
            
            # Get references to elements we'll need later
            self.compositor = self.pipeline.get_by_name("comp")
            self.right_source = self.pipeline.get_by_name("rightsrc")
            self.left_source = self.pipeline.get_by_name("leftsrc")
            
            # Set up bus for messages
            self.bus = self.pipeline.get_bus()
            self.bus.add_signal_watch()
            self.bus.connect("message", self.on_message)
            
            return True
            
        except Exception as e:
            print(f"Error creating pipeline: {e}")
            return False
    
    def on_message(self, bus, message):
        """Handle bus messages"""
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}")
            print(f"Debug info: {debug}")
            self.stop()
            
        elif message.type == Gst.MessageType.EOS:
            print("End of stream reached")
            self.stop()
            
        elif message.type == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                print(f"Pipeline state: {old_state.value_nick} -> {new_state.value_nick}")
                
        elif message.type == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            print(f"Warning: {warn}")
        
        elif message.type == Gst.MessageType.STREAM_START:
            print(f"Stream started from {message.src.get_name()}")
        
        elif message.type == Gst.MessageType.ASYNC_DONE:
            print("Pipeline async operations completed")
    
    def send_eos_to_right_source(self):
        """Send EOS event to right source pad to properly terminate it"""
        if self.right_source and not self.right_source_removed:
            print("Sending EOS to right source...")
            
            # Get the source pad of the right videotestsrc
            src_pad = self.right_source.get_static_pad("src")
            if src_pad:
                # Send EOS event to the pad
                eos_event = Gst.Event.new_eos()
                src_pad.send_event(eos_event)
                print("EOS sent to right source")
    
    def remove_right_source(self):
        """Remove the right video source from the pipeline"""
        if self.right_source_removed or not self.right_source or not self.compositor:
            return
            
        print("Removing right video source...")
        
        try:
            # First, send EOS to properly terminate the stream
            self.send_eos_to_right_source()
            
            # Wait a bit for EOS to propagate
            time.sleep(0.1)
            
            # Get the pads
            right_src_pad = self.right_source.get_static_pad("src")
            if right_src_pad:
                right_sink_pad = right_src_pad.get_peer()
                if right_sink_pad:
                    # Block the pad before unlinking to prevent data flow issues
                    def pad_block_callback(pad, info):
                        print("Pad blocked, proceeding with removal...")
                        
                        # Unlink the source from compositor
                        right_src_pad.unlink(right_sink_pad)
                        
                        # Release the compositor sink pad
                        self.compositor.release_request_pad(right_sink_pad)
                        
                        # Set right source to NULL state
                        self.right_source.set_state(Gst.State.NULL)
                        
                        # Remove from pipeline
                        self.pipeline.remove(self.right_source)
                        
                        return Gst.PadProbeReturn.REMOVE
                    
                    # Add probe to block the pad
                    right_src_pad.add_probe(
                        Gst.PadProbeType.BLOCK_DOWNSTREAM,
                        pad_block_callback
                    )
            
            self.right_source_removed = True
            print("Right video source removal initiated")
            
        except Exception as e:
            print(f"Error removing right source: {e}")
    
    def alternative_remove_method(self):
        """Alternative method: replace source with a dummy source"""
        if self.right_source_removed or not self.right_source:
            return
            
        print("Replacing right source with black screen...")
        
        try:
            # Simply change the pattern to black (pattern=2) instead of removing
            self.right_source.set_property("pattern", 2)  # Black screen
            
            # Or you could set it to a solid color
            # self.right_source.set_property("pattern", 17)  # Solid color
            # self.right_source.set_property("foreground-color", 0x000000FF)  # Black
            
            self.right_source_removed = True
            print("Right source replaced with black screen")
            
        except Exception as e:
            print(f"Error replacing right source: {e}")
    
    def removal_timer_thread(self):
        """Thread function to remove right source after 5 seconds"""
        time.sleep(5)
        if not self.right_source_removed:
            # Use the alternative method which is safer
            GLib.idle_add(self.alternative_remove_method)
            
            # Uncomment the line below and comment the line above to try the full removal method
            # GLib.idle_add(self.remove_right_source)
    
    def start(self):
        """Start the pipeline"""
        if not self.pipeline:
            if not self.create_pipeline_from_string():
                return False
        
        print("Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        
        if ret == Gst.StateChangeReturn.FAILURE:
            print("Unable to start pipeline")
            return False
        
        # Start the removal timer thread
        removal_thread = threading.Thread(target=self.removal_timer_thread)
        removal_thread.daemon = True
        removal_thread.start()
        
        print("Pipeline started successfully")
        print("Right source will be removed/replaced after 5 seconds...")
        print("Press Ctrl+C to stop")
        return True
    
    def stop(self):
        """Stop the pipeline"""
        if self.pipeline:
            print("Stopping pipeline...")
            self.pipeline.set_state(Gst.State.NULL)
        if self.loop:
            self.loop.quit()
    
    def run(self):
        """Run the pipeline with main loop"""
        if not self.start():
            return
        
        # Create and run main loop
        self.loop = GLib.MainLoop()
        
        try:
            self.loop.run()
        except KeyboardInterrupt:
            print("\nReceived interrupt signal")
        finally:
            self.stop()

def main():
    try:
        # Create and run the pipeline
        gst_pipeline = GStreamerPipelineWithRemoval()
        gst_pipeline.run()
        
    except Exception as e:
        print(f"Application error: {e}")

if __name__ == "__main__":
    main()