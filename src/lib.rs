//! sonoma-vision: a standalone AprilTag processor service for FRC SystemCore.
//!
//! The detector borrows heavily from napa-vision (Copyright (c) 2026
//! Zachary Wilson, <https://github.com/zwatx/NAPA>),
//! used under the GNU General Public License v3.0; see NOTICE.

pub mod capture;
pub mod clock;
pub mod config;
pub mod detector;
pub mod nt;
pub mod pose;
pub mod settings;
pub mod stream;
pub mod synth;
pub mod vision;
