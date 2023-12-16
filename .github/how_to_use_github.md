## Git・GitHubの使い方
- https://www.youtube.com/watch?v=WHwuNP4kalU

## 今回のプロジェクトでの開発の流れ
基本的にはGitHub Flowに従う
- https://docs.github.com/ja/get-started/quickstart/github-flow
- https://www.kagoya.jp/howto/it-glossary/develop/githubflow/

### 作業の流れ
1. mainブランチから作業用のブランチを切る
1. 1回でもcommitしたらpull requestを出す
    - 作業中のメモをpull request上に残すため
1. レビューしてほしい時にreviewerにメンションを飛ばす
    - バグが解決できずレビューしてほしい時でもメンションしてよい
    - reviewerとして設定された時ではなく，メンションされた時にレビューを開始する
1. approveされたらmainブランチにmergeする
   - approveするのはreviewr（レビューする人）
   - mergeするのはassignee（プルリクを出した人）

### 命名
- コミット
  - コマンドライン上で入力するので英語で書く（GitHub Desktopで入力しても良い）
  - [Conventional Commits](https://www.conventionalcommits.org/ja/v1.0.0/)に従っても良いが雑でよい
- ブランチ
  - コマンドライン上で入力するので英語で書く（GitHub Desktopで入力しても良い）
  - GitHub Flowでは`feature/hoge`とするが，今回は内容が伝われば良い
- プルリク
  - ブランチとプルリクは一対一で対応するが，ブラウザのGitHub上で書けるので分かりやすい日本語で書く

### プルリクエストに書くべきこと
これがあるとめっちゃ嬉しいってレベルなので，その時々必要ないと判断して削って良いです
#### プルリクの先頭
- 変更の概要
    - ひとことで
- 前提・目的・背景
    - なぜこのプルリクが必要なのか，みたいな話があれば書く
    - 関連するGoogle Drive上のリンクなどがあったらそれも貼る
- プルリクエストの変更範囲
    - ここまではこのプルリクで，こここから先は別のプルリクで，といった内容があれば書く
    - スコープが広すぎると判断したら途中で編集して良い
- (ReviewerとAssigneeを設定する)
#### レビュー時
- 変更の詳細
    - プルリクの先頭の概要よりは詳しめに
- 検証方法・検証結果
    - どのような設定・ハードウェアでコードを動かしてどんな結果が得られ，それが意図通りなのかどうか
- 特にレビューしてほしいこと
    - ChatGPTに一回聞いたうえで分からなかったら

### 注意点
- 参考にした資料があればリンクを貼る
  - 公式ドキュメントやサンプルコードなど
- レビューが必要なdiffのコミットを分ける
  - サンプルコードをそのまま持ってきたとき，コードフォーマッタをかけたときなどのコミットを独立させておく
