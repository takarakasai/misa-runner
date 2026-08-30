//! `misa-run` — 本体は [`misa_runner`] のライブラリ側。
//!
//! **この実行ファイルは Backend を持たない。** シリアル直結の機体
//! （namiashi）は組み込みなのでそのまま動くが、ブリッジ越しの機体は
//! **機体側のリポジトリの実行ファイル**を使う。そちらが自分の `Backend` を
//! 差して [`misa_runner::main_with`] を呼ぶ。

fn main() -> std::process::ExitCode {
    misa_runner::main_with(&[])
}
