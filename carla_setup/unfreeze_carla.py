import carla

def unfreeze():
    try:
        # Connect to the frozen server
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        world = client.get_world()

        # Force Synchronous Mode OFF
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

        print("✅ Success: CARLA is unfrozen and back in Asynchronous mode!")
    except Exception as e:
        print(f"❌ Failed to connect to CARLA: {e}")

if __name__ == '__main__':
    unfreeze()