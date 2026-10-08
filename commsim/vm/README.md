# commsim VM

The emulation needs root, kernel modules (`mac80211_hwsim`, `batman-adv`) and
apt ROS 2 Jazzy. The host is the Isaac machine (Ubuntu 25.10, no apt ROS), so
all of it lives in an Ubuntu 24.04 VM. Nothing here touches the host's ROS,
Isaac or conda setup.

## 1. Create the VM (host, needs sudo — run these yourself)

Multipass is the shortest path. Sizing is a placeholder until M1 measures the
per-robot cost (open question 8). Don't run sweeps while an Isaac session is
up — `scripts/sim_ctl.sh status` exits 4 when none is.

```bash
sudo snap install multipass
multipass launch 24.04 --name commsim --cpus 12 --memory 10G --disk 60G
multipass mount ~/SortBots commsim:/home/ubuntu/SortBots
multipass shell commsim
```

## 2. Provision (inside the VM)

```bash
cd ~/SortBots && sudo ./commsim/vm/provision_vm.sh
```

## 3. Build the message package (inside the VM)

```bash
bash -c 'source /opt/ros/jazzy/setup.bash && cd ~/SortBots/commsim/ws && colcon --log-base /tmp/colcon_log build'
```
