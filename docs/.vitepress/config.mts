import { defineConfig } from 'vitepress'

const ogImage = 'https://raw.githubusercontent.com/Ampoi/pylon/master/Assets/OGP.png'

export default defineConfig({
  lang: 'ja-JP',
  title: 'PyLoN Docs',
  description: 'PyLoN KSP mod と ROS2 bridge の起動・Topic・パーツ設定リファレンス',
  cleanUrls: true,
  lastUpdated: true,
  head: [
    ['meta', { name: 'theme-color', content: '#003dff' }],
    ['meta', { property: 'og:title', content: 'PyLoN Docs' }],
    ['meta', { property: 'og:description', content: 'KSP 1.x と ROS2 をつなぐセンサー・ロボティクスAPIドキュメント' }],
    ['meta', { property: 'og:type', content: 'website' }],
    ['meta', { property: 'og:image', content: ogImage }],
    ['meta', { property: 'og:image:type', content: 'image/png' }],
    ['meta', { property: 'og:image:width', content: '2400' }],
    ['meta', { property: 'og:image:height', content: '1260' }],
    ['meta', { property: 'og:image:alt', content: 'PyLoN' }],
    ['meta', { name: 'twitter:card', content: 'summary_large_image' }],
    ['meta', { name: 'twitter:image', content: ogImage }],
    ['meta', { name: 'twitter:image:alt', content: 'PyLoN' }]
  ],
  markdown: {
    lineNumbers: true
  },
  themeConfig: {
    siteTitle: 'PyLoN Docs',
    nav: [
      { text: '導入ガイド', link: '/guide/getting-started' },
      { text: 'デモ', link: '/demos/' },
      { text: 'APIリファレンス', link: '/api/topics' },
      { text: '設定・運用', link: '/reference/bridge-options' },
      { text: '本体への貢献', link: '/contributing/' }
    ],
    sidebar: [
      {
        text: '導入・アプリケーション開発',
        items: [
          { text: 'Getting Started', link: '/guide/getting-started' },
          { text: 'システム概要', link: '/guide/overview' },
          { text: 'ROS2アプリケーションを作る', link: '/guide/application-development' },
        ]
      },
      {
        text: 'デモ',
        items: [
          { text: 'デモ一覧・共通準備', link: '/demos/' },
          { text: '軌道上のデブリ周回・撮影', link: '/demos/debris-orbit' },
          { text: '2D LiDARとSLAM', link: '/demos/lidar-slam' },
          { text: '月面Nav2', link: '/demos/mun-nav2' }
        ]
      },
      {
        text: 'APIリファレンス',
        items: [
          { text: 'Topic一覧', link: '/api/topics' },
          { text: '機体制御とGround Truth', link: '/api/vehicle-control' },
          { text: 'Active vesselモデル', link: '/api/vessel-model' }
        ]
      },
      {
        text: 'パーツ別API',
        items: [
          { text: '2D / 3D LiDAR', link: '/parts/lidar' },
          { text: 'RGBカメラ', link: '/parts/camera' },
          { text: 'スタートラッカー', link: '/parts/star-tracker' },
          { text: 'サーボ / リニアモーター', link: '/parts/motors' },
          { text: 'KSP標準ホイール', link: '/parts/wheels' },
          { text: 'エンジン / RCS', link: '/parts/propulsion' },
          { text: 'デカプラー / フェアリング', link: '/parts/separation' },
          { text: 'ドッキングポート', link: '/parts/docking' }
        ]
      },
      {
        text: '設定・運用',
        items: [
          { text: 'Bridge起動オプション', link: '/reference/bridge-options' },
          { text: 'パーツ設定', link: '/reference/part-config' },
          { text: 'トラブルシュート', link: '/reference/troubleshooting' }
        ]
      },
      {
        text: 'PyLoN本体への貢献',
        items: [
          { text: '開発・変更の手順', link: '/contributing/' },
          { text: 'アーキテクチャ', link: '/contributing/architecture' },
          { text: '設計と検証の観点', link: '/contributing/design' },
          { text: 'ソース案内', link: '/contributing/source-map' }
        ]
      }
    ],
    outline: { level: [2, 3], label: 'このページ' },
    lastUpdated: { text: '最終更新' },
    docFooter: { prev: '前へ', next: '次へ' },
    returnToTopLabel: 'ページ上部へ',
    sidebarMenuLabel: 'メニュー',
    darkModeSwitchLabel: 'テーマ',
    lightModeSwitchTitle: 'ライトテーマへ',
    darkModeSwitchTitle: 'ダークテーマへ',
    search: {
      provider: 'local',
      options: {
        translations: {
          button: { buttonText: '検索', buttonAriaLabel: 'ドキュメントを検索' },
          modal: {
            noResultsText: '該当する結果がありません',
            resetButtonTitle: '検索をリセット',
            footer: { selectText: '選択', navigateText: '移動', closeText: '閉じる' }
          }
        }
      }
    },
    footer: {
      message: 'PyLoNを使ったROS2アプリケーション開発のためのガイドとAPIリファレンス',
      copyright: 'PyLoN'
    }
  }
})
