# Deployment Guide

This guide explains how to deploy the Physical AI & Humanoid Robotics book to GitHub Pages.

## Prerequisites

- GitHub account
- Git installed locally
- Node.js 18+ and npm installed
- Repository forked or cloned

## Quick Start

### 1. Update Configuration

Edit `docusaurus.config.js` and update the following fields:

```javascript
module.exports = {
  // ...
  url: 'https://YOUR-GITHUB-USERNAME.github.io',
  baseUrl: '/ai-humanoid-robotics-book/',
  organizationName: 'YOUR-GITHUB-USERNAME',
  projectName: 'ai-humanoid-robotics-book',
  // ...
};
```

Replace `YOUR-GITHUB-USERNAME` with your actual GitHub username.

### 2. Test Build Locally

Before deploying, ensure the site builds successfully:

```bash
# Install dependencies
npm install

# Build the site
npm run build

# Test locally
npm run serve
# Visit http://localhost:3000/ai-humanoid-robotics-book/
```

### 3. Enable GitHub Pages

1. Push your repository to GitHub:
   ```bash
   git add .
   git commit -m "Prepare for deployment"
   git push origin main
   ```

2. Go to your repository on GitHub
3. Navigate to **Settings** → **Pages**
4. Under "Source", select **Deploy from a branch**
5. Select branch: **gh-pages**
6. Select folder: **/ (root)**
7. Click **Save**

### 4. Automatic Deployment with GitHub Actions

The repository includes a GitHub Actions workflow (`.github/workflows/deploy.yml`) that automatically deploys on every push to `main`.

**How it works:**
1. On push to `main` branch
2. Workflow installs Node.js and dependencies
3. Builds the Docusaurus site
4. Deploys to `gh-pages` branch
5. GitHub Pages serves from `gh-pages`

**First-time setup:**
1. Ensure Actions are enabled: **Settings** → **Actions** → **General** → Allow all actions
2. Push to `main` branch
3. Check workflow status: **Actions** tab
4. Wait 2-5 minutes for deployment
5. Visit: `https://YOUR-USERNAME.github.io/ai-humanoid-robotics-book/`

### 5. Verify Deployment

After deployment completes:

1. Visit your site URL
2. Check all navigation works
3. Verify code examples render correctly
4. Test on mobile and desktop

## Manual Deployment (Alternative)

If you prefer manual deployment without GitHub Actions:

```bash
# Build the site
npm run build

# Deploy to gh-pages branch
GIT_USER=YOUR-GITHUB-USERNAME npm run deploy
```

This uses Docusaurus's built-in deployment command.

## Troubleshooting

### Build Fails Locally

**Issue**: `npm run build` fails with errors

**Solution**:
```bash
# Clear cache and reinstall
rm -rf node_modules package-lock.json
npm install
npm run build
```

### GitHub Action Fails

**Issue**: Workflow fails in Actions tab

**Solution**:
1. Check the error log in Actions tab
2. Common issues:
   - Incorrect Node version: Ensure workflow uses Node 18+
   - Missing dependencies: Verify `package.json` is committed
   - Build errors: Fix locally first, then push

### 404 Page After Deployment

**Issue**: Site shows 404 error

**Solution**:
1. Verify `baseUrl` in `docusaurus.config.js` matches repo name
2. Ensure GitHub Pages is set to deploy from `gh-pages` branch
3. Wait 5 minutes for DNS propagation
4. Clear browser cache

### CSS/Images Not Loading

**Issue**: Page loads but styling/images missing

**Solution**:
1. Check `baseUrl` is correct (must start and end with `/`)
2. Verify `url` uses HTTPS
3. Check browser console for 404 errors on assets

### Broken Internal Links

**Issue**: Links between pages don't work

**Solution**:
1. Use relative links: `[text](./other-page)`
2. Don't use absolute URLs for internal pages
3. Rebuild and test locally first

## Custom Domain (Optional)

To use a custom domain instead of `username.github.io`:

### 1. Update Configuration

```javascript
// docusaurus.config.js
module.exports = {
  url: 'https://your-custom-domain.com',
  baseUrl: '/', // Change to root
  // ...
};
```

### 2. Add CNAME File

Create `static/CNAME` with your domain:

```
your-custom-domain.com
```

### 3. Configure DNS

Add DNS records at your domain provider:

**For apex domain (example.com):**
```
A     @     185.199.108.153
A     @     185.199.109.153
A     @     185.199.110.153
A     @     185.199.111.153
```

**For subdomain (docs.example.com):**
```
CNAME docs your-username.github.io
```

### 4. Enable in GitHub

1. Go to **Settings** → **Pages**
2. Enter your custom domain
3. Enable **Enforce HTTPS**
4. Wait for DNS propagation (up to 24 hours)

## Performance Optimization

### 1. Enable Compression

GitHub Pages automatically serves gzip-compressed files.

### 2. Minimize Build Size

```bash
# Check build size
du -sh build/

# Optimize images before committing
# Use tools like ImageOptim, TinyPNG
```

### 3. Enable Service Worker (Optional)

Uncomment in `docusaurus.config.js`:

```javascript
module.exports = {
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          // ...
        },
        // Enable offline support
        serviceWorker: {
          swCustom: require.resolve('./src/sw.js'),
        },
      },
    ],
  ],
};
```

## Monitoring

### Analytics (Optional)

Add Google Analytics or other tracking:

```javascript
// docusaurus.config.js
module.exports = {
  themeConfig: {
    // ...
    gtag: {
      trackingID: 'G-XXXXXXXXXX',
      anonymizeIP: true,
    },
  },
};
```

### Uptime Monitoring

Use services like:
- UptimeRobot (free)
- Pingdom
- StatusCake

## Backup Strategy

1. **Repository**: Always kept on GitHub
2. **Build artifacts**: Available in `gh-pages` branch
3. **Local backup**:
   ```bash
   git clone --mirror https://github.com/YOUR-USERNAME/ai-humanoid-robotics-book.git
   ```

## Continuous Updates

### Workflow for Updates

1. Make changes locally
2. Test with `npm run build && npm run serve`
3. Commit and push to `main`
4. GitHub Actions automatically deploys
5. Verify changes at live URL

### Rollback Procedure

If deployment breaks the site:

```bash
# Revert to previous commit
git revert HEAD
git push origin main

# Or force push previous version
git reset --hard HEAD~1
git push --force origin main
```

## Security

### Secrets Management

- **Never commit API keys** to repository
- Use GitHub Secrets for sensitive data
- Review `.gitignore` to exclude:
  - `.env` files
  - `node_modules/`
  - Build artifacts

### Dependabot

Enable Dependabot for security updates:

1. **Settings** → **Security** → **Dependabot**
2. Enable **Dependabot security updates**
3. Enable **Dependabot version updates**

## Cost

**GitHub Pages is free** for public repositories:
- Unlimited bandwidth
- HTTPS included
- Custom domain supported
- 100GB soft limit on site size

## Support

If you encounter issues:

1. Check [Docusaurus documentation](https://docusaurus.io/docs/deployment#deploying-to-github-pages)
2. Review [GitHub Pages docs](https://docs.github.com/en/pages)
3. Search [GitHub Discussions](https://github.com/facebook/docusaurus/discussions)
4. Open an issue in this repository

## Testing Checklist

Before deploying, verify:

- [ ] Site builds without errors: `npm run build`
- [ ] All pages load correctly
- [ ] Navigation works (sidebar, links)
- [ ] Code blocks have syntax highlighting
- [ ] Images and diagrams display
- [ ] External links open in new tab
- [ ] Mobile responsive design works
- [ ] Search functionality works (if enabled)
- [ ] No console errors in browser
- [ ] Fast page load times

## Production Checklist

Before going live:

- [ ] Update `docusaurus.config.js` with correct URL
- [ ] Replace placeholder GitHub username
- [ ] Test build locally
- [ ] Review all content for accuracy
- [ ] Verify all links work
- [ ] Check spelling and grammar
- [ ] Optimize images
- [ ] Enable HTTPS on custom domain (if used)
- [ ] Set up analytics (optional)
- [ ] Configure custom 404 page (optional)
- [ ] Add social media meta tags (optional)

---

**Congratulations!** Your Physical AI & Humanoid Robotics book is now live. 🚀

Share your site:
- Twitter/X: "Check out my robotics book at [URL]"
- LinkedIn: Post link with project description
- Reddit: r/robotics, r/ROS
- Academic networks: ResearchGate, Academia.edu

For questions or contributions, open an issue or pull request.
