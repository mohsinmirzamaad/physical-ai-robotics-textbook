# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Feature**: 001-physical-ai-textbook
**Date**: 2026-03-01
**Status**: Complete

## Overview

This guide provides step-by-step instructions for setting up and running the Physical AI textbook platform locally and deploying to production.

## Prerequisites

### Required Software

- **Node.js**: v18.0 or higher ([Download](https://nodejs.org/))
- **Python**: 3.11 or higher ([Download](https://www.python.org/))
- **Git**: Latest version ([Download](https://git-scm.com/))
- **Package Manager**: npm (comes with Node.js) or yarn

### Required Accounts & API Keys

1. **OpenAI API Key** ([Get API Key](https://platform.openai.com/api-keys))
   - Used for: Chatbot responses, embeddings, translation
   - Estimated cost: $10-20 for development/testing

2. **Neon Postgres** ([Sign Up](https://neon.tech/))
   - Free tier: 512 MB storage
   - Used for: User accounts, sessions, preferences

3. **Qdrant Cloud** ([Sign Up](https://cloud.qdrant.io/))
   - Free tier: 1 GB storage
   - Used for: Vector embeddings for RAG

4. **Vercel Account** ([Sign Up](https://vercel.com/))
   - Free tier: Unlimited deployments
   - Used for: Hosting static site and API

5. **Email Service** (Optional for password reset)
   - Options: SendGrid, Resend, Mailgun
   - Free tiers available

## Project Structure

```
physical-ai-textbook/
├── docs/                          # Docusaurus content
│   ├── intro.md
│   ├── module-1-ros2/
│   ├── module-2-digital-twin/
│   ├── module-3-nvidia-isaac/
│   └── module-4-vla/
├── src/
│   ├── components/                # React components
│   │   ├── ChatbotWidget.tsx
│   │   ├── SignupForm.tsx
│   │   ├── LoginForm.tsx
│   │   └── ProtectedChapter.tsx
│   ├── lib/
│   │   ├── auth.ts               # Better-Auth config
│   │   └── auth-client.ts        # React hooks
│   ├── css/
│   │   └── custom.css
│   └── theme/
│       └── Root.tsx              # SessionProvider wrapper
├── api/                          # FastAPI backend
│   ├── main.py                   # FastAPI app
│   ├── routes/
│   │   ├── auth.py
│   │   ├── chat.py
│   │   └── content.py
│   ├── services/
│   │   ├── embeddings.py
│   │   ├── qdrant_client.py
│   │   └── openai_client.py
│   └── models/
│       └── schemas.py
├── scripts/
│   ├── embed-content.py          # Generate embeddings
│   └── seed-database.py          # Seed chapters
├── docusaurus.config.js
├── sidebars.js
├── package.json
├── requirements.txt              # Python dependencies
└── .env.example
```

## Setup Instructions

### Step 1: Clone Repository

```bash
git clone https://github.com/your-username/physical-ai-textbook.git
cd physical-ai-textbook
```

### Step 2: Environment Configuration

Create `.env` file in project root:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# OpenAI
OPENAI_API_KEY=sk-...

# Neon Postgres
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Better-Auth
AUTH_SECRET=generate-random-32-char-string
AUTH_URL=http://localhost:3000

# Email Service (Optional)
SENDGRID_API_KEY=your-sendgrid-key
FROM_EMAIL=noreply@yourdomain.com

# Redis (Optional, for caching)
REDIS_URL=redis://localhost:6379
```

**Generate AUTH_SECRET:**
```bash
node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
```

### Step 3: Install Dependencies

**Frontend (Docusaurus):**
```bash
npm install
```

**Backend (FastAPI):**
```bash
cd api
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
cd ..
```

### Step 4: Database Setup

**Create Neon Database:**
1. Go to [Neon Console](https://console.neon.tech/)
2. Create new project: "physical-ai-textbook"
3. Copy connection string to `.env`

**Run Migrations:**
```bash
# Install Better-Auth CLI
npm install -g better-auth

# Run migrations
npx better-auth migrate
```

**Seed Database:**
```bash
python scripts/seed-database.py
```

This creates chapter metadata from your docs/ directory.

### Step 5: Generate Embeddings

**Chunk and embed textbook content:**
```bash
python scripts/embed-content.py
```

This process:
1. Reads all markdown files from docs/
2. Chunks content (500-800 tokens, 100-150 overlap)
3. Generates embeddings via OpenAI
4. Stores vectors in Qdrant
5. Stores metadata in Postgres

**Estimated time:** 10-15 minutes for full textbook
**Estimated cost:** $2-5 (OpenAI embeddings)

### Step 6: Run Development Servers

**Terminal 1 - Frontend (Docusaurus):**
```bash
npm start
```
Opens at: http://localhost:3000

**Terminal 2 - Backend (FastAPI):**
```bash
cd api
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn main:app --reload --port 8000
```
API at: http://localhost:8000

**Terminal 3 - Redis (Optional, for caching):**
```bash
redis-server
```

### Step 7: Verify Setup

**Check Frontend:**
- Visit http://localhost:3000
- Navigate through chapters
- Verify content displays correctly

**Check Backend:**
- Visit http://localhost:8000/docs (FastAPI Swagger UI)
- Test `/health` endpoint
- Verify all services show "healthy"

**Test Authentication:**
1. Click "Sign Up" on homepage
2. Create test account
3. Verify email/password validation
4. Login and check session persistence

**Test Chatbot:**
1. Open any chapter
2. Click chatbot icon
3. Ask: "What is ROS 2?"
4. Verify response within 3 seconds
5. Select text and ask question about it

## Development Workflow

### Adding New Content

1. **Create markdown file** in appropriate module directory:
   ```bash
   touch docs/module-1-ros2/week-3-ros2-fundamentals/new-topic.md
   ```

2. **Write content** following template:
   ```markdown
   ---
   sidebar_position: 3
   ---

   # Topic Title

   ## Learning Outcomes
   - Outcome 1
   - Outcome 2

   ## Content
   [Your content here]

   ## Code Examples
   ```python
   # Example code
   ```

   ## Assessment
   1. Question 1
   2. Question 2
   ```

3. **Re-generate embeddings:**
   ```bash
   python scripts/embed-content.py --chapter new-topic
   ```

4. **Test chatbot** can answer questions about new content

### Running Tests

**Frontend Tests:**
```bash
npm test
```

**Backend Tests:**
```bash
cd api
pytest
```

**Integration Tests:**
```bash
npm run test:integration
```

### Code Quality

**Linting:**
```bash
# Frontend
npm run lint

# Backend
cd api
flake8 .
black .
```

**Type Checking:**
```bash
# Frontend
npm run typecheck

# Backend
cd api
mypy .
```

## Deployment

### Deploy to Vercel

**Prerequisites:**
- Vercel account
- Vercel CLI installed: `npm i -g vercel`

**Step 1: Configure Vercel**

Create `vercel.json`:
```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "framework": "docusaurus",
  "rewrites": [
    {
      "source": "/api/:path*",
      "destination": "/api/:path*"
    }
  ],
  "env": {
    "OPENAI_API_KEY": "@openai-api-key",
    "DATABASE_URL": "@database-url",
    "QDRANT_URL": "@qdrant-url",
    "QDRANT_API_KEY": "@qdrant-api-key",
    "AUTH_SECRET": "@auth-secret"
  }
}
```

**Step 2: Add Environment Variables**

```bash
vercel env add OPENAI_API_KEY
vercel env add DATABASE_URL
vercel env add QDRANT_URL
vercel env add QDRANT_API_KEY
vercel env add AUTH_SECRET
```

**Step 3: Deploy**

```bash
# Preview deployment
vercel

# Production deployment
vercel --prod
```

**Step 4: Update AUTH_URL**

After deployment, update `.env` and Vercel environment:
```env
AUTH_URL=https://your-project.vercel.app
```

### Deploy Backend to Vercel Serverless

Create `api/index.py` (Vercel entry point):
```python
from main import app

# Vercel serverless function handler
def handler(request):
    return app(request)
```

### Post-Deployment Checklist

- [ ] Verify site loads at production URL
- [ ] Test signup/login flow
- [ ] Test chatbot on multiple chapters
- [ ] Test personalization feature
- [ ] Test Urdu translation
- [ ] Check all API endpoints return 200
- [ ] Verify SSL certificate active
- [ ] Test mobile responsiveness
- [ ] Check performance (Lighthouse score >90)
- [ ] Monitor error logs for 24 hours

## Monitoring & Maintenance

### Health Checks

**API Health:**
```bash
curl https://your-project.vercel.app/api/health
```

**Database Connection:**
```bash
psql $DATABASE_URL -c "SELECT COUNT(*) FROM users;"
```

**Qdrant Status:**
```bash
curl -X GET "$QDRANT_URL/collections/textbook_embeddings" \
  -H "api-key: $QDRANT_API_KEY"
```

### Logs

**Vercel Logs:**
```bash
vercel logs
```

**Application Logs:**
- View in Vercel Dashboard > Logs
- Filter by: errors, warnings, API routes

### Performance Monitoring

**Key Metrics:**
- Page load time: <2s
- Chatbot response time: <3s
- API p95 latency: <500ms
- Error rate: <1%

**Tools:**
- Vercel Analytics (built-in)
- Google Lighthouse (performance audits)
- OpenAI Usage Dashboard (API costs)

### Database Maintenance

**Cleanup Expired Sessions (daily):**
```sql
DELETE FROM sessions WHERE expires_at < NOW();
```

**Cleanup Old Chat Messages (weekly):**
```sql
DELETE FROM chat_messages WHERE created_at < NOW() - INTERVAL '30 days';
```

**Backup Database:**
- Neon provides automatic backups
- Manual backup: `pg_dump $DATABASE_URL > backup.sql`

## Troubleshooting

### Common Issues

**Issue: Chatbot not responding**
- Check OpenAI API key is valid
- Verify Qdrant connection
- Check embeddings exist: `python scripts/embed-content.py --verify`

**Issue: Authentication not working**
- Verify DATABASE_URL is correct
- Check Better-Auth migrations ran
- Verify AUTH_SECRET is set

**Issue: Content not displaying**
- Check markdown files exist in docs/
- Verify sidebars.js configuration
- Clear build cache: `npm run clear && npm run build`

**Issue: Slow performance**
- Enable Redis caching
- Check Vercel region (should be closest to users)
- Optimize images: `npm run optimize-images`

### Debug Mode

**Enable verbose logging:**
```env
DEBUG=true
LOG_LEVEL=debug
```

**Frontend debug:**
```bash
npm start -- --debug
```

**Backend debug:**
```bash
uvicorn main:app --reload --log-level debug
```

## Cost Estimates

### Development (per month)
- OpenAI API: $10-20 (embeddings + chat)
- Neon Postgres: $0 (free tier)
- Qdrant Cloud: $0 (free tier)
- Vercel: $0 (free tier)
- **Total: $10-20/month**

### Production (100 users/day)
- OpenAI API: $50-100 (chat queries)
- Neon Postgres: $0-19 (may need paid tier)
- Qdrant Cloud: $0 (free tier sufficient)
- Vercel: $0-20 (may need Pro for bandwidth)
- **Total: $50-140/month**

## Support & Resources

### Documentation
- [Docusaurus Docs](https://docusaurus.io/docs)
- [Better-Auth Docs](https://better-auth.com/docs)
- [FastAPI Docs](https://fastapi.tiangolo.com/)
- [OpenAI API Docs](https://platform.openai.com/docs)
- [Qdrant Docs](https://qdrant.tech/documentation/)

### Community
- GitHub Issues: [Report bugs](https://github.com/your-username/physical-ai-textbook/issues)
- Discussions: [Ask questions](https://github.com/your-username/physical-ai-textbook/discussions)

## Next Steps

After completing setup:

1. **Content Creation**: Write textbook chapters for all 13 weeks
2. **Testing**: Create comprehensive test suite
3. **Optimization**: Implement caching and performance improvements
4. **Analytics**: Add user analytics and learning insights
5. **Feedback**: Gather user feedback and iterate

## Appendix

### Useful Commands

```bash
# Development
npm start                    # Start Docusaurus dev server
npm run build               # Build for production
npm run serve               # Serve production build locally
npm run clear               # Clear cache

# Backend
uvicorn main:app --reload   # Start FastAPI dev server
pytest                      # Run tests
black .                     # Format code
flake8 .                    # Lint code

# Database
npx better-auth migrate     # Run migrations
psql $DATABASE_URL          # Connect to database

# Deployment
vercel                      # Deploy preview
vercel --prod              # Deploy production
vercel logs                # View logs

# Embeddings
python scripts/embed-content.py              # Embed all content
python scripts/embed-content.py --chapter X  # Embed specific chapter
python scripts/embed-content.py --verify     # Verify embeddings
```

### Environment Variables Reference

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| OPENAI_API_KEY | Yes | OpenAI API key | sk-... |
| DATABASE_URL | Yes | Neon Postgres connection | postgresql://... |
| QDRANT_URL | Yes | Qdrant Cloud URL | https://... |
| QDRANT_API_KEY | Yes | Qdrant API key | ... |
| AUTH_SECRET | Yes | Better-Auth secret (32 chars) | ... |
| AUTH_URL | Yes | Application URL | http://localhost:3000 |
| SENDGRID_API_KEY | No | Email service key | SG... |
| FROM_EMAIL | No | Sender email | noreply@... |
| REDIS_URL | No | Redis connection | redis://... |
| DEBUG | No | Enable debug mode | true |
| LOG_LEVEL | No | Logging level | debug |

---

**Last Updated**: 2026-03-01
**Version**: 1.0.0
