use std::{fmt, marker::PhantomData};

use serde::{
    de::{self, Visitor},
    Deserialize, Deserializer,
};

/// Useful deserializer function that will initialise a `null` value (such as a YAML field
/// that is not given an explicit value) to `Some(T::default())` instead of `None`.
///
/// # Example
///
/// ```yaml
/// # config.yaml
/// devices:
///  ```
/// This evaluates to the JSON object `{ "devices": null }`, this deserializer will initialise
/// `devices` to `Some(Devices::default())` instead of `None`.
pub fn deserialise_empty_to_default<'de, T, D>(deserializer: D) -> Result<Option<T>, D::Error>
where
    T: Deserialize<'de> + Default,
    D: Deserializer<'de>,
{
    struct OptionalStructVisitor<T>(PhantomData<T>);

    impl<'de, T> Visitor<'de> for OptionalStructVisitor<T>
    where
        T: Deserialize<'de> + Default,
    {
        type Value = Option<T>;

        fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
            formatter.write_str("null, map, or empty map")
        }

        // Handle null case (field not present)
        fn visit_none<E>(self) -> Result<Self::Value, E>
        where
            E: de::Error,
        {
            Ok(None)
        }

        // Handle empty map case (field present with no values)
        fn visit_unit<E>(self) -> Result<Self::Value, E>
        where
            E: de::Error,
        {
            Ok(Some(T::default()))
        }

        // Handle regular map case (field present with values)
        fn visit_map<M>(self, map: M) -> Result<Self::Value, M::Error>
        where
            M: de::MapAccess<'de>,
        {
            match Deserialize::deserialize(de::value::MapAccessDeserializer::new(map)) {
                Ok(value) => Ok(Some(value)),
                Err(_) => Ok(Some(T::default())),
            }
        }
    }

    deserializer.deserialize_any(OptionalStructVisitor(PhantomData))
}
