//! An example serializing and deserializing protobufs using the `protobuf` and `math` feature flag.

use nt_client::{Client, data::{DataType, NetworkTableData}, math::{Pose2d, Rotation2d, Translation2d}, protobuf::{Protobuf, ProtobufData}, subscribe::ReceivedMessage, topic::Properties};

#[tokio::main]
async fn main() {
    let client = Client::new(Default::default());

    client.connect_setup(setup).await.unwrap();
}

fn setup(client: &Client) {
    let mut schema_manager = client.schema_manager();
    let pub_topic = client.topic("/mypose");
    tokio::spawn(async move {
        // publish proto schema so the server can understand it
        // a more in-depth example can be found in the `schema` example
        schema_manager.publish_proto::<Pose2d>().await.unwrap();

        // publish pose2d
        // set it to retained since the publisher will be dropped after the value is set
        let publisher = pub_topic.publish(Properties { retained: Some(true), ..Default::default() }).await.unwrap();

        let pose = Pose2d {
            translation: Translation2d { x: 1.3, y: -0.5 },
            rotation: Rotation2d { value: 1.283 },
        };
        publisher.set(pose.into_protobuf_data()).await.unwrap();
    });

    let sub_topic = client.topic("/serverpose");
    tokio::spawn(async move {
        // subscribe and get arm feedforward constants
        let mut subscriber = sub_topic.subscribe(Default::default()).await.unwrap();

        while let Ok(message) = subscriber.recv().await {
            if let ReceivedMessage::Updated((topic, value)) = message {
                match topic.r#type() {
                    // check to make sure the data type is correct
                    DataType::Protobuf(type_name) if type_name == &Pose2d::proto_type_name() => {
                        let pose = <Protobuf<Pose2d>>::from_value(value).expect("valid pose2d struct").0;
                        println!("Got pose: {pose:?}");
                    }
                    _ => println!("Not a pose2d"),
                }
            }
        }
    });
}

