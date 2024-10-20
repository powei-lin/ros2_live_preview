use futures::StreamExt;
use image::ImageReader;
use image::RgbImage;
use ros2_client::ros2::policy;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use show_image::create_window;
use show_image::glam::UVec2;
use show_image::WindowOptions;
use std::io::Cursor;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub sec: i32,
    pub nanosec: u32,
    pub frame_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RawImage {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub encoding: String,
    pub is_bigendian: u8,
    pub step: u32,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompressedImage {
    pub header: Header,
    pub format: String,
    pub data: Vec<u8>,
}

pub trait PreviewImage {
    fn to_image(&self) -> image::DynamicImage;
    fn as_str() -> &'static str;
}
impl PreviewImage for RawImage {
    fn to_image(&self) -> image::DynamicImage {
        match self.encoding.as_str() {
            "bgr8" => {
                let mut bgr =
                    RgbImage::from_raw(self.width, self.height, self.data.clone()).unwrap();
                bgr.pixels_mut().for_each(|p| {
                    p.0.reverse();
                });
                image::DynamicImage::ImageRgb8(bgr)
            }
            _ => {
                panic!()
            }
        }
    }

    fn as_str() -> &'static str {
        "Image"
    }
}
impl PreviewImage for CompressedImage {
    fn to_image(&self) -> image::DynamicImage {
        ImageReader::new(Cursor::new(self.data.clone()))
            .with_guessed_format()
            .unwrap()
            .decode()
            .unwrap()
    }

    fn as_str() -> &'static str {
        "CompressedImage"
    }
}

fn live_preview<T: DeserializeOwned + PreviewImage + 'static>(topic_name: &str) {
    let context = ros2_client::Context::new().unwrap();
    let mut node = context
        .new_node(
            ros2_client::NodeName::new("/rustdds", "rustdds_listener").unwrap(),
            ros2_client::NodeOptions::new().enable_rosout(false),
        )
        .unwrap();

    let reliable_qos = ros2_client::ros2::QosPolicyBuilder::new()
        .history(policy::History::KeepLast { depth: 2 })
        .reliability(policy::Reliability::Reliable {
            max_blocking_time: ros2_client::ros2::Duration::from_millis(100),
        })
        .durability(policy::Durability::Volatile)
        .build();
    let chatter_topic = node
        .create_topic(
            &ros2_client::Name::new("/", topic_name).unwrap(),
            ros2_client::MessageTypeName::new("sensor_msgs", T::as_str()),
            &ros2_client::DEFAULT_SUBSCRIPTION_QOS,
        )
        .unwrap();

    let chatter_subscription = node
        .create_subscription::<T>(&chatter_topic, Some(reliable_qos))
        .unwrap();

    let options = WindowOptions {
        preserve_aspect_ratio: true,
        start_hidden: true,
        ..Default::default()
    };
    let window = create_window(topic_name, options).unwrap();

    let subscription_stream = chatter_subscription
        .async_stream()
        .for_each(|result| async {
            match result {
                Ok((msg, _info)) => {
                    let img = msg.to_image();
                    let window_w = 1280;
                    let window_h = img.height() * window_w / img.width();

                    window.run_function(move |mut w| {
                        if w.image_info().is_none() {
                            w.set_inner_size(UVec2::new(window_w, window_h));
                            w.set_visible(true);
                            println!("init");
                        }
                    });
                    window.set_image(topic_name, msg.to_image()).unwrap();
                }
                Err(e) => eprintln!("Receive request error: {:?}", e),
            }
        });

    smol::block_on(subscription_stream);
}

pub fn main() {
    let topic_name = "ssbu_c";
    show_image::run_context(move || live_preview::<CompressedImage>(topic_name));
}
