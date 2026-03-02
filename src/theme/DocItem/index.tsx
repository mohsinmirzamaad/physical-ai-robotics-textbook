import React, { useState } from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type { WrapperProps } from '@docusaurus/types';
import styles from './styles.module.css';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): JSX.Element {
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [showOriginal, setShowOriginal] = useState(true);

  // Get the current page content
  const getPageContent = () => {
    const article = document.querySelector('article');
    return article?.innerText || '';
  };

  // Get the current page title
  const getPageTitle = () => {
    const title = document.querySelector('h1');
    return title?.innerText || 'Chapter';
  };

  const handlePersonalize = async () => {
    setIsPersonalizing(true);
    setShowOriginal(false);

    try {
      const content = getPageContent();
      const title = getPageTitle();

      const response = await fetch('/api/content/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content,
          user_id: 'demo-user', // TODO: Get from auth session
          chapter_title: title,
        }),
      });

      if (!response.ok) {
        throw new Error('Personalization failed');
      }

      const data = await response.json();
      setPersonalizedContent(data.content);
    } catch (error) {
      console.error('Personalization error:', error);
      alert('Failed to personalize content. Please try again.');
      setShowOriginal(true);
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleTranslate = async () => {
    setIsTranslating(true);
    setShowOriginal(false);

    try {
      const content = getPageContent();

      const response = await fetch('/api/content/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content,
          target_language: 'urdu',
        }),
      });

      if (!response.ok) {
        throw new Error('Translation failed');
      }

      const data = await response.json();
      setTranslatedContent(data.content);
    } catch (error) {
      console.error('Translation error:', error);
      alert('Failed to translate content. Please try again.');
      setShowOriginal(true);
    } finally {
      setIsTranslating(false);
    }
  };

  const handleShowOriginal = () => {
    setShowOriginal(true);
    setPersonalizedContent(null);
    setTranslatedContent(null);
  };

  return (
    <>
      {/* Action Buttons */}
      <div className={styles.actionBar}>
        <button
          className={styles.actionButton}
          onClick={handlePersonalize}
          disabled={isPersonalizing || isTranslating}
        >
          {isPersonalizing ? '⏳ Personalizing...' : '✨ Personalize for Me'}
        </button>
        <button
          className={styles.actionButton}
          onClick={handleTranslate}
          disabled={isPersonalizing || isTranslating}
        >
          {isTranslating ? '⏳ Translating...' : '🌐 Translate to Urdu'}
        </button>
        {!showOriginal && (
          <button className={styles.resetButton} onClick={handleShowOriginal}>
            ↩️ Show Original
          </button>
        )}
      </div>

      {/* Original Content */}
      {showOriginal && <DocItem {...props} />}

      {/* Personalized Content */}
      {!showOriginal && personalizedContent && (
        <div className={styles.processedContent}>
          <div className={styles.badge}>✨ Personalized for Your Level</div>
          <div className={styles.contentWrapper}>
            <pre className={styles.content}>{personalizedContent}</pre>
          </div>
        </div>
      )}

      {/* Translated Content */}
      {!showOriginal && translatedContent && (
        <div className={styles.processedContent}>
          <div className={styles.badge}>🌐 Translated to Urdu</div>
          <div className={styles.contentWrapper}>
            <pre className={styles.content} dir="rtl">
              {translatedContent}
            </pre>
          </div>
        </div>
      )}
    </>
  );
}
