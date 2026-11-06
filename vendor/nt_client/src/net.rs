use std::{collections::HashMap, time::Duration};

use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Visitor, ser::Error};

use crate::{data::{DataType, NetworkTableData}, subscribe::SubscriptionOptions, topic::Properties};

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ServerboundMessage {
    Text(ServerboundTextData),
    Binary(BinaryData),
    Ping,
}

#[derive(Serialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "lowercase", tag = "method", content = "params")]
pub(crate) enum ServerboundTextData {
    Publish(Publish),
    Unpublish(Unpublish),
    SetProperties(SetProperties),

    Subscribe(Subscribe),
    Unsubscribe(Unsubscribe),
}

#[derive(Serialize, Debug, Clone, PartialEq, Eq)]
pub(crate) struct Publish {
    pub name: String,
    pub pubuid: i32,
    pub r#type: DataType,
    pub properties: Properties,
}

#[derive(Serialize, Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Unpublish {
    pub pubuid: i32,
}

#[derive(Serialize, Debug, Clone, PartialEq, Eq)]
pub(crate) struct SetProperties {
    pub name: String,
    pub update: HashMap<String, Option<serde_json::Value>>,
}

#[derive(Serialize, Debug, Clone, PartialEq, Eq)]
pub(crate) struct Subscribe {
    pub topics: Vec<String>,
    pub subuid: i32,
    pub options: SubscriptionOptions,
}

#[derive(Serialize, Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Unsubscribe {
    pub subuid: i32,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ClientboundData {
    Text(ClientboundTextData),
    Binary(BinaryData),
}

#[derive(Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "lowercase", tag = "method", content = "params")]
pub(crate) enum ClientboundTextData {
    Announce(Announce),
    Unannounce(Unannounce),
    Properties(PropertiesData),
}

#[derive(Deserialize, Debug, Clone, PartialEq, Eq)]
pub(crate) struct Announce {
    pub name: String,
    pub id: i32,
    pub r#type: DataType,
    pub pubuid: Option<i32>,
    pub properties: Properties,
}

#[derive(Deserialize, Debug, Clone, PartialEq, Eq)]
pub(crate) struct Unannounce {
    pub name: String,
    pub id: i32,
}

#[derive(Deserialize, Debug, Clone, PartialEq, Eq)]
pub(crate) struct PropertiesData {
    pub name: String,
    // NOTE: this doesn't seem to ever exist
    pub ack: Option<bool>,
    pub update: HashMap<String, Option<serde_json::Value>>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub(crate) struct BinaryData {
    pub id: i32,
    #[serde(serialize_with = "serialize_dur_as_micros", deserialize_with = "deserialize_micros_as_dur")]
    pub timestamp: Duration,
    #[serde(serialize_with = "serialize_data_type_as_u32", deserialize_with = "deserialize_u32_as_data_type")]
    pub data_type: DataType,
    pub data: rmpv::Value,
}

impl BinaryData {
    pub fn new<T: NetworkTableData>(id: i32, timestamp: Duration, data: T) -> Self {
        Self { id, timestamp, data_type: T::data_type(), data: data.into_value() }
    }
}

fn serialize_dur_as_micros<S>(duration: &Duration, serializer: S) -> Result<S::Ok, S::Error>
where S: Serializer
{
    serializer.serialize_u64(duration.as_micros().try_into().map_err(S::Error::custom)?)
}

fn deserialize_micros_as_dur<'de, D>(deserializer: D) -> Result<Duration, D::Error>
where D: Deserializer<'de>
{
    deserializer.deserialize_u64(DurationMicrosVisitor)
}

struct DurationMicrosVisitor;

impl Visitor<'_> for DurationMicrosVisitor {
    type Value = Duration;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(formatter, "a valid micros duration")
    }

    fn visit_i64<E>(self, v: i64) -> Result<Self::Value, E>
    where E: serde::de::Error
    {
        self.visit_u64(v.try_into().map_err(E::custom)?)
    }

    fn visit_u64<E>(self, v: u64) -> Result<Self::Value, E>
    where E: serde::de::Error
    {
        Ok(Duration::from_micros(v))
    }
}

fn serialize_data_type_as_u32<S>(data_type: &DataType, serializer: S) -> Result<S::Ok, S::Error>
where S: Serializer
{
    serializer.serialize_u32(data_type.as_id())
}

fn deserialize_u32_as_data_type<'de, D>(deserializer: D) -> Result<DataType, D::Error>
where D: Deserializer<'de>
{
    deserializer.deserialize_u32(DataTypeVisitor)
}

struct DataTypeVisitor;

impl Visitor<'_> for DataTypeVisitor {
    type Value = DataType;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(formatter, "a valid type id")
    }

    fn visit_i64<E>(self, v: i64) -> Result<Self::Value, E>
    where E: serde::de::Error
    {
        self.visit_u64(v.try_into().map_err(E::custom)?)
    }

    fn visit_u64<E>(self, v: u64) -> Result<Self::Value, E>
    where E: serde::de::Error
    {
        DataType::from_id(v.try_into().map_err(E::custom)?).ok_or(E::custom(format!("{v} is not a valid type id")))
    }
}

