# Deployment Guide

## Quick Start: Deploy to GitHub Pages

Follow these steps to deploy your Physical AI textbook to GitHub Pages:

### Step 1: Create GitHub Repository

1. Go to [GitHub](https://github.com) and create a new repository
2. Name it `hackathon-1` (or any name you prefer)
3. Make it **Public** (required for free GitHub Pages)
4. Don't initialize with README (we already have one)

### Step 2: Update Configuration

Before pushing, update `docusaurus.config.ts` with your GitHub username:

```typescript
// Replace 'your-username' with your actual GitHub username
url: 'https://your-username.github.io',
baseUrl: '/hackathon-1/',  // Use your repo name
organizationName: 'your-username',
projectName: 'hackathon-1',
```

### Step 3: Push to GitHub

```bash
# Add remote (replace YOUR-USERNAME with your GitHub username)
git remote add origin https://github.com/YOUR-USERNAME/hackathon-1.git

# Push to GitHub
git push -u origin 001-physical-ai-textbook

# Or if you want to push to main branch:
git checkout -b main
git merge 001-physical-ai-textbook
git push -u origin main
```

### Step 4: Enable GitHub Pages

1. Go to your repository on GitHub
2. Click **Settings** → **Pages**
3. Under "Build and deployment":
   - Source: Select **GitHub Actions**
4. The deployment will start automatically

### Step 5: Wait for Deployment

- Go to **Actions** tab in your repository
- You'll see the "Deploy to GitHub Pages" workflow running
- Wait for it to complete (usually 2-3 minutes)
- Your site will be live at: `https://your-username.github.io/hackathon-1/`

## Alternative: Deploy to Vercel

Vercel is easier and faster for deployment:

### Step 1: Push to GitHub

```bash
git remote add origin https://github.com/YOUR-USERNAME/hackathon-1.git
git push -u origin 001-physical-ai-textbook
```

### Step 2: Deploy to Vercel

1. Go to [Vercel](https://vercel.com)
2. Sign in with GitHub
3. Click **Add New** → **Project**
4. Import your `hackathon-1` repository
5. Configure:
   - Framework Preset: **Docusaurus**
   - Build Command: `npm run build`
   - Output Directory: `build`
6. Click **Deploy**

Your site will be live at: `https://hackathon-1-your-username.vercel.app`

## Troubleshooting

### Build Fails on GitHub Actions

**Issue**: Build fails with "Module not found" error

**Solution**: Make sure all dependencies are in `package.json`:
```bash
npm install
git add package.json package-lock.json
git commit -m "fix: update dependencies"
git push
```

### 404 Error on GitHub Pages

**Issue**: Site shows 404 error

**Solution**: Check `docusaurus.config.ts`:
- `baseUrl` must match your repository name: `/hackathon-1/`
- `url` must be: `https://your-username.github.io`

### Broken Links

**Issue**: Internal links don't work

**Solution**: Use relative paths in markdown:
```markdown
[Link to chapter](./chapter-name.md)
```

## Testing Locally

Before deploying, test the production build locally:

```bash
# Build
npm run build

# Serve
npm run serve

# Open http://localhost:3000/hackathon-1/
```

## Updating the Site

After making changes:

```bash
git add .
git commit -m "docs: update content"
git push

# GitHub Actions will automatically rebuild and deploy
```

## Custom Domain (Optional)

To use a custom domain with GitHub Pages:

1. Add a `CNAME` file to `static/` folder:
   ```
   your-domain.com
   ```

2. Update `docusaurus.config.ts`:
   ```typescript
   url: 'https://your-domain.com',
   baseUrl: '/',
   ```

3. Configure DNS settings with your domain provider

## Next Steps

After deployment:

1. ✅ Test all navigation links
2. ✅ Verify all 16 chapters are accessible
3. ✅ Check mobile responsiveness
4. ✅ Test search functionality
5. ✅ Submit to hackathon form with live URL

## Hackathon Submission

Submit your project at: https://forms.gle/CQsSEGM3GeCrL43c8

Include:
1. ✅ Public GitHub Repo Link
2. ✅ Published Book Link (GitHub Pages or Vercel)
3. ⏳ Demo video (under 90 seconds)
4. ⏳ WhatsApp number

## Support

If you encounter issues:
- Check GitHub Actions logs for build errors
- Review Docusaurus documentation: https://docusaurus.io/docs/deployment
- Ensure all files are committed and pushed
