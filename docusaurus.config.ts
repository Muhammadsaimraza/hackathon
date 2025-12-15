import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'From Digital Brains to Physical Bodies',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://muhammadsaimraza.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/hackathon/',

  // GitHub pages deployment config.
  organizationName: 'Muhammadsaimraza', // Usually your GitHub org/user name.
  projectName: 'hackathon', // Usually your repo name.
 
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',


  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          editUrl:
            'https://github.com/Muhammadsaimraza/',
        },
        blog: false, // Disable the blog plugin.
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Enforce dark mode
    // colorMode: {
    //   defaultMode: 'dark',
    //   disableSwitch: true,
    //   respectPrefersColorScheme: false,
    // },
    // Replace with your project's social card
    image: 'img/download.jpg',
    navbar: {
      title: 'ROBOLEARN',
      logo: {
        alt: 'RoboLearn Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Learn Free',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/Muhammadsaimraza',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Course',
          items: [
            {
              label: 'Textbook',
              to: '/docs/Introduction/', // Updated to the first page of the textbook
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus', // Placeholder
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/docusaurus', // Placeholder
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Panaversity',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Saim.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    algolia: {
      // Add your Algolia Search API keys here
      appId: 'YOUR_APP_ID',
      apiKey: 'YOUR_SEARCH_API_KEY',
      indexName: 'YOUR_INDEX_NAME',
      contextualSearch: true,
    },

  } satisfies Preset.ThemeConfig,
};

export default config;