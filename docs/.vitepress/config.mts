import { defineConfig } from 'vitepress'

export default defineConfig({
  lang: 'ja-JP',
  title: 'Kerbal ROS2 API',
  description: 'KerbalLiDAR KSP mod と ROS2 bridge の起動・Topic・パーツ設定リファレンス',
  cleanUrls: true,
  lastUpdated: true,
  head: [
    ['meta', { name: 'theme-color', content: '#071b2f' }],
    ['meta', { property: 'og:title', content: 'Kerbal ROS2 API' }],
    ['meta', { property: 'og:description', content: 'KSP 1.x と ROS2 をつなぐセンサー・ロボティクスAPIドキュメント' }]
  ],
  markdown: {
    lineNumbers: true
  },
  themeConfig: {
    siteTitle: 'Kerbal ROS2 API',
    nav: [
      { text: 'ガイド', link: '/guide/overview' },
      { text: 'Topic API', link: '/api/topics' },
      { text: 'パーツ', link: '/parts/lidar' },
      { text: '機体制御', link: '/api/vehicle-control' },
      { text: 'リファレンス', link: '/reference/bridge-options' }
    ],
    sidebar: [
      {
        text: 'はじめに',
        items: [
          { text: 'システム概要', link: '/guide/overview' },
          { text: '起動手順', link: '/guide/getting-started' },
          { text: '2D LiDAR MappingとNav2', link: '/guide/nav2' }
        ]
      },
      {
        text: 'API',
        items: [
          { text: 'Topic一覧', link: '/api/topics' },
          { text: '機体制御とGround Truth', link: '/api/vehicle-control' },
          { text: 'Active vesselモデル', link: '/api/vessel-model' }
        ]
      },
      {
        text: 'パーツ別',
        items: [
          { text: '2D / 3D LiDAR', link: '/parts/lidar' },
          { text: 'RGBカメラ', link: '/parts/camera' },
          { text: 'サーボ / リニアモーター', link: '/parts/motors' },
          { text: 'KSP標準ホイール', link: '/parts/wheels' },
          { text: 'エンジン / RCS', link: '/parts/propulsion' },
          { text: 'デカプラー / フェアリング', link: '/parts/separation' },
          { text: 'ドッキングポート', link: '/parts/docking' }
        ]
      },
      {
        text: 'リファレンス',
        items: [
          { text: 'Bridge起動オプション', link: '/reference/bridge-options' },
          { text: 'パーツ設定', link: '/reference/part-config' },
          { text: 'トラブルシュート', link: '/reference/troubleshooting' }
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
      message: 'KSP mod と ROS2 bridge の実装に基づくリファレンス',
      copyright: 'Kerbal LiDAR Lab'
    }
  }
})
