# The Network Development Package
This package contains functionality that is useful for developing and testing custom networks. Namely, there are two things that this package does: architecture testing and anchor finding.

# Anchor Finding
YOLO networks use anchor boxes that assist with accurate predictions and having those right can be crucial for making training easier on your network. All you need for this is to have all your raw data formatted properly (see `datamanager/README.md`) in some folder like `raw-data-folder`. Then you need to know the `resize-dim` you are using and the `number-of-anchors` you would like to find. Then from the terminal you can generate the correct anchors like so:
```bash
MS-CV2 user$ python3 -m netdev F [raw-data-folder] [resize-dim] [number-of-anchors]
```
For example this might look like
```bash
MS-CV2 user$ python3 -m netdev F data-test 256 9
--- calculated best anchors ---
10,14  41,40  29,81  46,142  114,97  80,194  67,248  195,147  130,249
```
You can copy the line with the anchors and paste it directly into the network config file.

# Architecture Testing
When developing network architectures it can be helpful to see how the layers interact with each other and how the data shape changes between them. This is what the architecture testing functionality does. Visually runs through your network layer by layer. If your config file is at `cfg/network.cfg` and you want to test up to layer `N` with a batch size of `B` then you can run such a test with the terminal like so:
```bash
MS-CV2 user$ python3 -m netdev T --stop_at N --batch_size B cfg/network.cfg
```
Of course, the `--stop_at` and `--batch_size` arguments are not required for such testing.