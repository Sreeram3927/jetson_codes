from vision_system import VisionSystem


def main():
    vision = VisionSystem()
    try:
        vision.setup()
        vision.run()
    except KeyboardInterrupt:
        vision.stop()


if __name__ == "__main__":
    main()
