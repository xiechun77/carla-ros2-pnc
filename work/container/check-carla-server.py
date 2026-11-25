import json
import time
import carla

last_error = None
for attempt in range(30):
    try:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(3.0)
        server_version = client.get_server_version()
        world = client.get_world()
        first = world.wait_for_tick(5.0)
        second = world.wait_for_tick(5.0)
        result = {
            'client_version': client.get_client_version(),
            'server_version': server_version,
            'map': world.get_map().name,
            'frames': [first.frame, second.frame],
            'elapsed_seconds': [first.timestamp.elapsed_seconds, second.timestamp.elapsed_seconds],
            'synchronous_mode': world.get_settings().synchronous_mode,
            'available_maps': client.get_available_maps(),
        }
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if server_version != client.get_client_version() or second.frame <= first.frame:
            raise SystemExit('Version mismatch or simulation frames did not advance')
        break
    except RuntimeError as exc:
        last_error = str(exc)
        time.sleep(1)
else:
    raise SystemExit(last_error)
