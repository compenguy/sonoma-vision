//! An example serializing and deserializing binary "structs" using the `struct` and `math` feature flag.

use nt_client::{Client, data::{DataType, NetworkTableData}, math::ArmFeedforward, r#struct::{Struct, StructData}, subscribe::ReceivedMessage, topic::Properties};

#[tokio::main]
async fn main() {
    let client = Client::new(Default::default());

    client.connect_setup(setup).await.unwrap()
}

fn setup(client: &Client) {
    let mut schema_manager = client.schema_manager();
    let pub_topic = client.topic("/myfeedforward");
    tokio::spawn(async move {
        // publish struct schema so the server can understand it
        // a more in-depth example can be found in the `schema` example
        schema_manager.publish_struct::<ArmFeedforward>().await.unwrap();

        // publish arm feedforward constants (raw binary)
        // set it to retained since the publisher will be dropped after the value is set
        let publisher = pub_topic.publish(Properties { retained: Some(true), ..Default::default() }).await.unwrap();

        let feedforward = ArmFeedforward {
            k_s: 8.2,
            k_g: 3.2,
            k_v: 11.5,
            k_a: 9.22,
            d_t: 0.1,
        };
        publisher.set(feedforward.into_struct_data()).await.unwrap();
    });

    let sub_topic = client.topic("/serverfeedforward");
    tokio::spawn(async move {
        // subscribe and get arm feedforward constants
        let mut subscriber = sub_topic.subscribe(Default::default()).await.unwrap();

        while let Ok(message) = subscriber.recv().await {
            if let ReceivedMessage::Updated((topic, value)) = message {
                match topic.r#type() {
                    // check to make sure the data type is correct
                    DataType::Struct(type_name) if type_name == &ArmFeedforward::struct_type_name() => {
                        let feedforward = <Struct<ArmFeedforward>>::from_value(value).expect("valid feedforward struct").0;
                        println!("Got feedforward: {feedforward:?}");
                    }
                    _ => println!("Not a feedforward"),
                }
            }
        }
    });
}

