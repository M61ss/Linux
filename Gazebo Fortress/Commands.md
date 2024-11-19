# Gazebo Fortress commands

## Open an SDF file

```shell
ign gazebo <path_to_file.sdf>
```

## Select Gazebo version

To ensure to run a file with a specific version of gazebo, run:

```shell
ign gazebo --force-version 6.0.0 <path_to_file.sdf>
```

## Publish data on a topic

[Open in Gazebo your SDF file](#open-an-sdf-file).

> [!NOTE]
>
> Remember to press the play button in the Gazebo GUI. 

```shell
ign topic -t "<topic_name>" -m <message_type> -p "<content>"
```

#### Options

- `-t "<topic_name>"`: specifies the topic to publish to.
- `-m <message_type>`: specifies the message type.
- `-p "<content>"`: specifies the content (value) of the message.
