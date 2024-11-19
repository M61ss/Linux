# Gazebo Fortress usage summary

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

1. [Open in Gazebo your SDF file](#open-an-sdf-file).

   > [!NOTE]
   >
   > Remember to press the play button in the Gazebo GUI. 

2. ```shell
   ign topic -t "<topic_name>" -m <message_type> -p "<content>"
   ```

#### Options

- `-t "<topic_name>"`: specifies the topic to publish to.
- `-m <message_type>`: specifies the message type.
- `-p "<content>"`: specifies the content (value) of the message.

### Publish from keyboard: `KeyPublisher`

[Open in Gazebo your SDF file](#open-an-sdf-file).

In the top right corner click on the plugins dropdown list (vertical ellipsis), click the "Key Publisher".

```shell
ign topic -e -t /keyboard/keypress
```

It will display all messages sent on `/keyboard/keypress` topic.

#### Options

- `-t "<topic_name>"`: specifies the topic to publish to.
- `-e`: for "echo", allows to display messages on terminal.
