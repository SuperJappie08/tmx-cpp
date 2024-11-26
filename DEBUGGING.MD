# Debug Flags
- `TMX_RX_DEBUG`: Debug message on recieving
- `TMX_TX_DEBUG`: Debug message on transmission

## Deprecated
Deprecated debug flags and there replacements:

|      Flag      | Description        | Replacement                                                           |
| :------------: | :----------------- | :-------------------------------------------------------------------- |
| `TMX_HW_DEBUG` | Set poolsize to 1. | Intialize the `tmx_cpp::TMX` object with `parse_pool_size` equal to 1 |
