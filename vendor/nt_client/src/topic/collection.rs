//! Collection of topics that can be used to subscribe to multiple topics at once.

use std::{fmt::Debug, iter::FusedIterator};

use crate::{ClientHandle, error::ConnectionClosedError, subscribe::{Subscriber, SubscriptionOptions}};

use super::Topic;

/// Represents a collection of topics.
///
/// This is used to subscribe to multiple topics at once.
///
/// # Examples
/// ```no_run
/// use nt_client::Client;
///
/// # tokio_test::block_on(async {
/// let client = Client::new(Default::default());
///
/// client.connect_setup(setup).await.unwrap();
/// # });
///
/// fn setup(client: &Client) {
///     let topics = client.topics(vec![
///         "/topic".to_owned(),
///         "/nested/topic".to_owned(),
///         "/deeply/nested/topic".to_owned(),
///     ]);
///     tokio::spawn(async move {
///         let subscriber = topics.subscribe(Default::default()).await;
///
///         // do something with subscriber...
///     });
/// }
/// ```
#[derive(Debug, Clone)]
pub struct TopicCollection {
    names: Vec<String>,
    handle: ClientHandle,
}

impl IntoIterator for TopicCollection {
    type Item = Topic;
    type IntoIter = IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        IntoIter::new(self)
    }
}

impl PartialEq for TopicCollection {
    fn eq(&self, other: &Self) -> bool {
        self.names == other.names
    }
}

impl Eq for TopicCollection { }

impl TopicCollection {
    pub(crate) fn new(
        names: Vec<String>,
        handle: ClientHandle,
    ) -> Self {
        Self { names, handle }
    }

    /// Returns a slice of topic names this collection contains.
    pub fn names(&self) -> &Vec<String> {
        &self.names
    }

    /// Returns a mutable slice of topic names this collection contains.
    pub fn names_mut(&mut self) -> &mut Vec<String> {
        &mut self.names
    }

    /// Subscribes to this collection of topics.
    ///
    /// This method does not require the [`Client`] websocket connection to be made.
    ///
    /// [`Client`]: crate::Client
    pub async fn subscribe(&self, options: SubscriptionOptions) -> Result<Subscriber, ConnectionClosedError> {
        Subscriber::new(self.names.clone(), options, self.handle.announced_topics.clone(), self.handle.server_send.clone(), self.handle.client_send.subscribe()).await
    }
}

/// Iterator that iterates over [`Topic`]s in a [`TopicCollection`].
///
/// This is obtained by the [`TopicCollection::into_iter`] method.
pub struct IntoIter {
    name_iter: std::vec::IntoIter<String>,
    handle: ClientHandle,
}

impl Iterator for IntoIter {
    type Item = Topic;

    fn next(&mut self) -> Option<Self::Item> {
        self.name_iter.next()
            .map(|name| Topic::new(name, self.handle.clone()))
    }
}

impl DoubleEndedIterator for IntoIter {
    fn next_back(&mut self) -> Option<Self::Item> {
        self.name_iter.next_back()
            .map(|name| Topic::new(name, self.handle.clone()))
    }
}

impl ExactSizeIterator for IntoIter {
    fn len(&self) -> usize {
        self.name_iter.len()
    }
}

impl FusedIterator for IntoIter { }

impl IntoIter {
    pub(self) fn new(collection: TopicCollection) -> Self {
        IntoIter {
            name_iter: collection.names.into_iter(),
            handle: collection.handle,
        }
    }
}

