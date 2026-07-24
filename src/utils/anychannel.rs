use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// Anything we can (asynchronously for now) send a value to.
pub trait AnySender<T> {
    fn anysend(&mut self, value: T) -> impl Future<Output = ()>;
}

impl<T, const N: usize> AnySender<T> for embassy_sync::channel::Sender<'_, CriticalSectionRawMutex, T, N> {
    async fn anysend(&mut self, value: T) {
        self.send(value).await;
    }
}

impl<T: Clone, const N: usize, const SUBS: usize, const PUBS: usize> AnySender<T>
    for embassy_sync::pubsub::Publisher<'_, CriticalSectionRawMutex, T, N, SUBS, PUBS>
{
    async fn anysend(&mut self, value: T) {
        self.publish(value).await;
    }
}

impl<T: Clone, const N: usize> AnySender<T> for embassy_sync::watch::Sender<'_, CriticalSectionRawMutex, T, N> {
    async fn anysend(&mut self, value: T) {
        self.send(value);
    }
}

/// Anything we can asynchronously receive a value from.
pub trait AnyReceiver<T> {
    fn anyreceive(&mut self) -> impl Future<Output = T>;
}

impl<T, const N: usize> AnyReceiver<T> for embassy_sync::channel::Receiver<'_, CriticalSectionRawMutex, T, N> {
    async fn anyreceive(&mut self) -> T {
        self.receive().await
    }
}

impl<T: Clone, const N: usize, const SUBS: usize, const PUBS: usize> AnyReceiver<T>
    for embassy_sync::pubsub::Subscriber<'_, CriticalSectionRawMutex, T, N, SUBS, PUBS>
{
    async fn anyreceive(&mut self) -> T {
        self.next_message_pure().await
    }
}

impl<T: Clone, const N: usize> AnyReceiver<T> for embassy_sync::watch::Receiver<'_, CriticalSectionRawMutex, T, N> {
    async fn anyreceive(&mut self) -> T {
        self.changed().await
    }
}
