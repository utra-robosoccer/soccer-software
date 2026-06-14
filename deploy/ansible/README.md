# deploy/ansible — fleet management

One command deploys the lean runtime image to every robot (blueprint §12).

```bash
# 1. Copy + edit the inventory (the .local.ini name is git-ignored).
cp inventory.example.ini inventory.local.ini

# 2. Deploy a specific image tag to the whole fleet.
ansible-playbook -i inventory.local.ini deploy.yml \
  -e image_tag=$(git rev-parse HEAD) \
  -e ghcr_user=$GHCR_USER -e ghcr_token=$GHCR_TOKEN
```

## Secrets

`ghcr_token` / Wi-Fi creds are passed via `--extra-vars` or Ansible Vault and
must **never** be committed or echoed to logs (blueprint §12). `inventory.local.ini`
and `*.env` are in `.gitignore`.

## Requirements

```bash
ansible-galaxy collection install community.docker
```
