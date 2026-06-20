try:
    from .bandwidth_app import main
except ImportError:
    from bandwidth_app import main


if __name__ == "__main__":
    main()
