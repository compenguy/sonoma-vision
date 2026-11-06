# Local patches to nt_client 0.5.1

Vendored from crates.io (MIT, https://github.com/DatAsianBoi123/nt_client) with four changes, each
marked `sonoma-vision patch` in the source. All are candidates for upstream pull requests.

1. `src/lib.rs`, `try_connect`: connect with `connect_async_with_config(.., disable_nagle: true)`.
   Without TCP_NODELAY, small NT4 messages can wait tens of milliseconds for the previous segment's
   ACK, which delays published values and inflates the round trip used for time sync.
2. `src/lib.rs`, `start_update_time_task`: estimate the server's current time as
   `timestamp + rtt / 2`; upstream subtracts, putting the client's server clock about one round trip
   behind.
3. `src/publish.rs`: `Publisher::set_at(value, timestamp)`, publishing a value stamped with an
   explicit server time (e.g. a camera frame's capture time) through the existing private `set_time`.
4. `src/publish.rs`, `GenericPublisher::new`: wait for the server's announcement of *this* publisher
   (matching `pubuid`). Upstream takes the first announcement with any `pubuid`, so publishers created
   concurrently, or while another task publishes, can adopt another topic's id and send their values
   to it.
