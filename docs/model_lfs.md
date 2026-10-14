# Downloaded model storage with Git LFS

The 22 CC BY 4.0 models live in `data/models/downloaded/`, approximately610MB combined. Their attribution is in that directory. Six downloaded NC/NC-SA models are excluded. These raw source assets are not yet configured as playable cars.

Only `data/models/downloaded/*.glb` is tracked through LFS. Existing generated project models and physics_sim models keep their existing storage. LFS stores a small SHA-256 pointer in Git and the full binary in LFS storage; working files are still ordinary GLBs.

## Publish the local commit

From the authoritative `S:\claude_code\racing_game` checkout:

```powershell
git lfs ls-files --size
git push origin main
```

The installed LFS pre-push hook uploads the referenced binaries automatically before pushing the Git commit. A successful Git push is required before the cloud checkout can access these new assets. GitHub LFS storage/bandwidth allowances and charges depend on the account; this setup does not alter billing settings.

## Other machines and cloud checkouts

Install Git LFS on that machine/container, then run from the repository root:

```bash
git lfs install --local
git lfs pull
```

Normal cloning with LFS installed generally downloads the objects automatically. The explicit pull handles environments that skipped LFS checkout. These commands can be added to the cloud environment setup script; the environment must have network access and access to this repository's GitHub LFS endpoint. If a cached cloud environment switches branches, its maintenance script should also pull current LFS objects. Start the task from a commit containing the asset import.

Check that the assets are binaries instead of pointer text:

```bash
git lfs ls-files --size
git lfs fsck
```

`git lfs ls-files` shows `*` for populated files and `-` for pointer files. GLB files start with `glTF`; pointer files start with `version https://git-lfs.github.com/spec/v1`.

## Adding another permitted model later

Copy its GLB into the same directory, add its creator/source/license to both attribution files, then:

```bash
git add data/models/downloaded
# Review the staged files and attribution before committing.
git commit -m "Add attributed car model"
git push
```

The directory rule already tracks new GLBs. Do not run `git lfs track "*.glb"` for this task: that would also change storage expectations for unrelated generated assets. Do not rewrite repository history to migrate existing files; this import adds new files.

A Git bundle includes LFS pointers, not LFS binaries. The existing `tools/export_dev_bundle.ps1` alone therefore does not provide these downloaded assets for offline transfer; use an online LFS pull, or separately transfer the downloaded model directory and validate its hashes.

Sources:
- https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage
- https://docs.github.com/en/repositories/working-with-files/managing-large-files/configuring-git-large-file-storage
- https://learn.chatgpt.com/docs/environments/cloud-environment
