# GitHub Pages 部署说明

本项目课程版是纯前端 Vite 应用，可以部署到 GitHub Pages。当前 `web/vite.config.ts` 已设置：

```ts
base: './'
```

因此打包后的资源使用相对路径，适合部署到 `https://用户名.github.io/仓库名/` 这种 GitHub Pages 子路径。

## 方式一：使用 GitHub Actions 自动部署

推荐这种方式。仓库每次 push 到 `main` 后，GitHub 自动安装依赖、构建 `web/dist`，并发布到 Pages。

1. 在仓库根目录创建 `.github/workflows/deploy-pages.yml`。
2. 写入以下内容：

```yaml
name: Deploy GitHub Pages

on:
  push:
    branches:
      - main
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: pages
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Setup Node
        uses: actions/setup-node@v4
        with:
          node-version: 22

      - name: Install dependencies
        working-directory: web
        run: npm ci

      - name: Build
        working-directory: web
        run: npm run build

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: web/dist

  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

3. 提交并 push 到 GitHub。
4. 打开仓库 Settings -> Pages。
5. Source 选择 GitHub Actions。
6. 等待 Actions 跑完，页面会发布到 Pages URL。

## 方式二：手动构建并发布 dist

如果不想配置 Actions，可以本地构建后把 `web/dist` 内容发布到 Pages 分支。

```powershell
cd web
npm.cmd install
npm.cmd run build
```

构建完成后，`web/dist` 就是静态网站目录。可以使用 `gh-pages` 工具发布：

```powershell
cd web
npx.cmd gh-pages -d dist
```

执行后，仓库会生成或更新 `gh-pages` 分支。然后在 GitHub 仓库 Settings -> Pages 中选择：

- Source: Deploy from a branch
- Branch: `gh-pages`
- Folder: `/root`

## API Key 注意事项

项目不会把 DashScope API Key 写进代码或 GitHub 仓库。页面中的 API Key 输入框只保存在浏览器 localStorage 中。部署到 GitHub Pages 后，使用者需要在页面右侧自行输入自己的 Key。

不要把真实 API Key 写入：

- 源代码
- README
- GitHub Actions workflow
- `.env` 并提交到仓库

## 部署前检查

建议每次部署前本地执行：

```powershell
cd web
npm.cmd test
npm.cmd run build
```

当前项目打包时可能出现 Vite 的 chunk size warning，这是 Three.js 体积导致的提示，不影响 GitHub Pages 部署。
