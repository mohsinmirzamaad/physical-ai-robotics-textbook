# Physical AI & Humanoid Robotics Textbook

A comprehensive textbook for teaching Physical AI and Humanoid Robotics, built with Docusaurus.

## 🚀 Live Demo

- **GitHub Pages**: [Coming Soon - Update after deployment]
- **Vercel**: [Coming Soon - Alternative deployment]

## 📚 Content Overview

This textbook covers 16 comprehensive chapters across 4 modules:

### Module 1: ROS 2 Fundamentals (Week 1-5)
- Physical AI foundations and embodied intelligence
- Sensor systems (Vision, LiDAR, IMU)
- ROS 2 architecture and communication patterns
- Launch files and system configuration

### Module 2: Digital Twin Simulation (Week 6-7)
- Gazebo physics simulation
- Unity integration for photorealistic rendering
- Sensor simulation and fusion

### Module 3: NVIDIA Isaac Platform (Week 8-10)
- Isaac SDK and ecosystem
- Isaac Sim for photorealistic simulation
- Isaac ROS for hardware-accelerated perception
- Nav2 for autonomous navigation

### Module 4: Vision-Language-Action (Week 11-13)
- Humanoid kinematics and control
- Bipedal locomotion and balance
- Conversational robotics with LLMs

## 🛠️ Tech Stack

- **Frontend**: Docusaurus 3.6.3 + React 18.2 + TypeScript
- **Backend**: FastAPI + OpenAI + Qdrant + Neon Postgres
- **Authentication**: Better-Auth 1.3.10
- **Deployment**: GitHub Pages / Vercel

## 📦 Installation

```bash
# Clone the repository
git clone https://github.com/your-username/hackathon-1.git
cd hackathon-1

# Install dependencies
npm install

# Start development server
npm start
```

## 🚀 Deployment

### GitHub Pages

1. Update `docusaurus.config.ts` with your GitHub username:
   ```typescript
   url: 'https://your-username.github.io',
   baseUrl: '/hackathon-1/',
   organizationName: 'your-username',
   projectName: 'hackathon-1',
   ```

2. Push to GitHub:
   ```bash
   git add .
   git commit -m "docs: update deployment configuration"
   git push origin main
   ```

3. Enable GitHub Pages in repository settings:
   - Go to Settings → Pages
   - Source: GitHub Actions
   - The site will automatically deploy on push

### Vercel (Alternative)

1. Import project to Vercel
2. Configure build settings:
   - Build Command: `npm run build`
   - Output Directory: `build`
3. Deploy

## 🏗️ Build

```bash
# Build for production
npm run build

# Serve locally
npm run serve
```

## 📝 Features

### Implemented ✅
- 16 comprehensive textbook chapters
- Multi-language support (English, Urdu)
- Syntax highlighting for Python, C++, YAML, Bash
- Responsive design
- Search functionality

### Backend Ready (Needs Frontend) ⏳
- RAG chatbot with OpenAI + Qdrant
- User authentication with Better-Auth
- Content personalization based on user background
- Urdu translation service

## 🎯 Hackathon Requirements

**Base Functionality (100 points):**
- ✅ Docusaurus textbook with comprehensive content
- ⏳ RAG chatbot (backend ready)

**Bonus Features (200 points possible):**
- ⏳ Better-Auth signup/signin (+50 points)
- ⏳ Content personalization (+50 points)
- ⏳ Urdu translation (+50 points)
- ⏳ Reusable Claude subagents (+50 points)

## 📂 Project Structure

```
hackathon-1/
├── docs/                          # Textbook content
│   ├── intro.md
│   ├── module-1-ros2/
│   ├── module-2-digital-twin/
│   ├── module-3-nvidia-isaac/
│   └── module-4-vla/
├── src/                           # React components
│   ├── components/
│   ├── lib/                       # Auth configuration
│   └── theme/                     # Custom theme
├── api/                           # FastAPI backend
│   ├── database/
│   ├── models/
│   └── services/
├── .github/workflows/             # GitHub Actions
├── docusaurus.config.ts           # Docusaurus configuration
├── sidebars.ts                    # Sidebar navigation
└── package.json
```

## 🤝 Contributing

This is a hackathon project. For questions or suggestions, please open an issue.

## 📄 License

MIT License

## 🙏 Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/)
- Powered by [Claude Code](https://claude.com/claude-code)
- Part of [Panaversity](https://panaversity.org) initiative

