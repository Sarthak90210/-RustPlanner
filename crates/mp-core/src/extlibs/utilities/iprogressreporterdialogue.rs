//! Port of `ExtLibs/Utilities/IProgressReporterDialogue.cs`

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// `delegate void DoWorkEventHandler(IProgressReporterDialogue sender)`
pub type DoWorkEventHandler = Arc<dyn Fn(&dyn IProgressReporterDialogue) + Send + Sync>;

/// `public interface IProgressReporterDialogue`
pub trait IProgressReporterDialogue: Send + Sync {
    /// `event DoWorkEventHandler DoWork` (+=)
    fn add_do_work(&self, handler: DoWorkEventHandler);

    /// `ProgressWorkerEventArgs doWorkArgs { get; set; }`
    fn do_work_args(&self) -> Arc<ProgressWorkerEventArgs>;
    fn set_do_work_args(&self, args: Arc<ProgressWorkerEventArgs>);

    fn dispose(&self);

    /// Called at setup - will kick off the background process on a thread pool thread
    fn run_background_operation_async(&self);

    /// Called from the BG thread; `progress` in %, -1 means indeterminate.
    fn update_progress_and_status(&self, progress: i32, status: &str);

    /// `BeginInvoke(Delegate method)`: run `method` on the dialogue's UI thread.
    fn begin_invoke(&self, method: Box<dyn FnOnce() + Send>);
}

/// `PropertyChangedEventHandler(object sender, PropertyChangedEventArgs e)`
pub type CancelRequestChangedHandler = Arc<dyn Fn(&ProgressWorkerEventArgs, &str) + Send + Sync>;

/// `public class ProgressWorkerEventArgs : EventArgs`, shared between the UI and the worker.
#[derive(Default)]
pub struct ProgressWorkerEventArgs {
    force_exit: AtomicBool,
    pub error_message: Mutex<Option<String>>,
    _cancel_requested: AtomicBool,
    pub cancel_acknowledged: AtomicBool,
    cancel_request_changed: Mutex<Vec<CancelRequestChangedHandler>>,
}

impl ProgressWorkerEventArgs {
    pub fn new() -> Self {
        Self::default()
    }

    /// `ForceExit`
    pub fn force_exit(&self) -> bool {
        self.force_exit.load(Ordering::SeqCst)
    }

    pub fn set_force_exit(&self, v: bool) {
        self.force_exit.store(v, Ordering::SeqCst);
    }

    /// `CancelRequested`
    pub fn cancel_requested(&self) -> bool {
        self._cancel_requested.load(Ordering::SeqCst)
    }

    /// The `CancelRequested` setter raises `CancelRequestChanged("CancelRequested")`.
    pub fn set_cancel_requested(&self, value: bool) {
        self._cancel_requested.store(value, Ordering::SeqCst);
        let handlers = self.cancel_request_changed.lock().unwrap_or_else(|e| e.into_inner()).clone();
        for h in handlers {
            h(self, "CancelRequested");
        }
    }

    /// `event PropertyChangedEventHandler CancelRequestChanged` (+=)
    pub fn add_cancel_request_changed(&self, h: CancelRequestChangedHandler) {
        self.cancel_request_changed.lock().unwrap_or_else(|e| e.into_inner()).push(h);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;

    #[test]
    fn cancel_raises_event() {
        let a = ProgressWorkerEventArgs::new();
        let n = Arc::new(AtomicUsize::new(0));
        let n2 = n.clone();
        a.add_cancel_request_changed(Arc::new(move |_, p| {
            assert_eq!(p, "CancelRequested");
            n2.fetch_add(1, Ordering::SeqCst);
        }));
        a.set_cancel_requested(true);
        assert!(a.cancel_requested());
        assert_eq!(n.load(Ordering::SeqCst), 1);
    }
}
