# deploy/ansible — fleet management

One command provisions and deploys the ZED stack to every robot (blueprint §12).

```bash
# 1. Copy + edit the inventory (the .local.ini name is git-ignored).
cp inventory.example.ini inventory.local.ini

# 2. ONE-TIME host provisioning per robot (CDI hook fix, mode=cdi, swap).
#    Required before the first deploy; idempotent, safe to re-run.
ansible-playbook -i inventory.local.ini provision.yml

# 3a. Deploy from the registry (CI / self-hosted arm64 runner pushes to GHCR).
ansible-playbook -i inventory.local.ini deploy.yml \
  -e image_tag=$(git rev-parse HEAD) \
  -e ghcr_user=$GHCR_USER -e ghcr_token=$GHCR_TOKEN

# 3b. …or deploy images built ON each robot with deploy/docker/Dockerfile.jetson.
ansible-playbook -i inventory.local.ini deploy.yml \
  -e pull_images=false -e registry= -e image_tag=jazzy
```

## Playbooks

| Playbook        | When           | What                                                                                                                                                                                                                                                                    |
| --------------- | -------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `provision.yml` | once per robot | Applies the host fixes proven during the live JetPack 7.2 / L4T R39.2 bring-up: disables the broken `enable-cuda-compat` CDI hook, forces the container runtime into `mode = "cdi"`, and adds an 8 GB swapfile for on-device builds (`docs/zed_jetson_integration.md`). |
| `deploy.yml`    | each release   | (Re)starts the two-container stack — a GPU `soccer-camera` container (ZED SDK + zed-ros2-wrapper) and the CPU `soccer-app` container that bridges the ZED topics onto the generic camera contract.                                                                      |

> The Jetson images are **not** built in cloud CI (a 19 GB arm64 image with a
> licensed ZED SDK download under QEMU emulation is impractical). Build them on a
> robot (or a self-hosted arm64 runner) with `deploy/docker/Dockerfile.jetson`;
> cloud CI keeps building the lean CPU `Dockerfile.runtime` for sim.

## Secrets

`ghcr_token` / Wi-Fi creds are passed via `--extra-vars` or Ansible Vault and
must **never** be committed or echoed to logs (blueprint §12). `inventory.local.ini`
and `*.env` are in `.gitignore`.

## Requirements

```bash
ansible-galaxy collection install community.docker
```
