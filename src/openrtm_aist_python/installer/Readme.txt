OpenRTM-aist-Python Windows インストーラー作成ツールの解説

　作成日： 2010/3/19 
　作成者： 白田

  改訂: 2010/04/24 栗原

●　目次
1. 前提条件
2. ディレクトリ構成
3. ファイル構成
4. インストール動作
5. アンインストール動作


●　説明
1. 前提条件

　　本ツールは、Windows Installer XML(WiX)を使用して、msiファイルを
　　作成するものです。

　　本ツールを実行するには、下記ソフトウエアがインストールされている事を
　　前提とします。

　　　・Python2.4, 2.5, 2.6 の何れか
　　　・PyYAML-3.09.win32-py2.4, 2.5, 2.6 の何れか
　　　・WiX3.5 Toolset
　　　・環境変数の「PATH」と「PYTHONPATH」に、使用するPython情報が設定
　　　　されている事。


2. ディレクトリ

　　ローカルPCに、以下のディレクトリ構成でインストール対象ファイルが
　　準備してある事を前提としています。

　　C:\distribution
　　　　│
　　　　├─ OpenRTM-aist-Python-1.0.0
　　　　│　　├─ doc　　※doxygen
　　　　│　　│　　├─ ClassReference-jp
　　　　│　　│　　└─ ClassReference-en
　　　　│　　│
　　　　│　　├─ idlcompiler ※ idlcompiler
　　　　│　　│　　├─ idlcompile.bat 
　　　　│　　│　　└─ idlcompile.py
　　　　│　　│
　　　　│　　└─ OpenRTM_aist　　※sdist
　　　　│　　　　　├─ *.py(ソースファイル)
　　　　│　　　　　├─ examples
　　　　│　　　　　├─ RTM_IDL
　　　　│　　　　　└─ utils
　　　　│　　　　　　　　├─ rtcd
　　　　│　　　　　　　　├─ rtcprof
　　　　│　　　　　　　　├─ rtc-template
　　　　│　　　　　　　　└─ rtm-naming
　　　　│
　　　　│
　　　　├─ omniORBpy-3.0-Python2.4
　　　　│　　├─ bin
　　　　│　　│　　└─ x86_win32
　　　　│　　└─ lib
　　　　│　　　　　├─ Python
　　　　│　　　　　└─ x86_win32
　　　　│
　　　　│
　　　　├─ omniORBpy-3.4-Python2.5
　　　　│　　├─ bin
　　　　│　　│　　└─ x86_win32
　　　　│　　└─ lib
　　　　│　　　　　├─ Python
　　　　│　　　　　└─ x86_win32
　　　　│
　　　　│
　　　　├─ omniORBpy-3.4-Python2.6
　　　　  　　├─ bin
　　　　  　　│　　└─ x86_win32
　　　　  　　└─ lib
　　　　  　　　　　├─ Python
　　　　  　　　　　└─ x86_win32


　　(1) OpenRTM-aist-Python-1.0.0 は、
　　　　Python版OpenRTM-aistのインストール対象ファイルであり、
　　　　doxygenにより作成されたリファレンスファイル(※doxygen)と、
        IDLコンパイルを行うためのファイル(※idlcompiler)と、
　　　　python setup.py sdist により作成されたファイル(※sdist)を含む。

　　(2) omniORBpy-3.0-Python2.4 は、
　　　　Python2.4用omniORBpy-3.0である。

　　(3) omniORBpy-3.4-Python2.5 は、
　　　　Python2.5用omniORBpy-3.4である。

　　(4) omniORBpy-3.4-Python2.6 は、
　　　　Python2.6用omniORBpy-3.4である。


　　※上記構成は、msi作成バッチ build.cmd で定義しており、変更可能です。
　　　変更する場合、build.cmd, OpenRTMpywxs.py, omniORBpy24wxs.py, 
　　　omniORBpy25wxs.py, omniORBpy26wxs.py の整合を取って下さい。


3. ファイル構成

　　[このディレクトリ]
　　　　│
　　　　├─ Bitmaps
　　　　│　　├─ bannrbmp.bmp 　インストール用バナー画像　※1
　　　　│　　└─ dlgbmp.bmp 　　インストール用ダイアログ画像　※1
　　　　├─ License.rtf　　　　　ライセンス　※1
　　　　├─ WiLangId.vbs 　　　　ユーティリティ　※1
　　　　├─ WiSubStg.vbs 　　　　ユーティリティ　※1
　　　　├─ langs.txt　　　　　　ランゲージ一覧　※1
　　　　├─ makewxs.py 　　　　　wxsdファイルジェネレータ　※1
　　　　├─ uuid.py　　　　　　　UUIDジェネレータ　※1
　　　　├─ yat.py 　　　　　　　Yamlテンプレートプロセッサ　※1
　　　　│
　　　　│
　　　　├─ build.cmd　　　msi作成バッチ
　　　　├─ cleanup.cmd　　テンポラリファイル削除バッチ
　　　　│
　　　　├─ OpenRTM-aist-Python.wxs.in　OpenRTM-aist用WiXテンプレート
　　　　├─ omniORBpy24_inc.wxs.in　　　Python2.4用WiXテンプレート
　　　　├─ omniORBpy25_inc.wxs.in　　　Python2.5用WiXテンプレート
　　　　├─ omniORBpy26_inc.wxs.in　　　Python2.6用WiXテンプレート
　　　　│
　　　　├─ OpenRTMpywxs.py　　　OpenRTM-aist用wxsファイルジェネレータ
　　　　├─ omniORBpy24wxs.py　　Python2.4用wxsファイルジェネレータ
　　　　├─ omniORBpy25wxs.py　　Python2.5用wxsファイルジェネレータ
　　　　├─ omniORBpy26wxs.py　　Python2.6用wxsファイルジェネレータ
　　　　│
　　　　├─ idlcompile.bat　　　IDLコンパイル起動バッチ
　　　　├─ idlcompile.py 　　　IDLコンパイルスクリプト
　　　　│
　　　　└─ WixUI_**-**.wxl　　　各国語メッセージローカライズ　※2

　　　※1は、C++版よりコピーしたもの。

　　　※2は、WixUI_Mondo_RTM.wxs 用に、インストールの種類を選択する画面で
　　　　表示するメッセージを、オリジナルのローカライズファイルを元に、
　　　　InstallScopeDlgDescription のメッセージを
　　　　SetupTypeDlgDescription よりコピーしています。

　　[ビルド後に使用するファイル]
　　　　│
　　　　├─ OpenRTM-aist-Python-1.0.0.msi　英語のインストーラー
　　　　│
　　　　└─ OpenRTM-aist-Python-1.0.0_**-**.msi　各国語毎のインストーラー

　　　※build.cmd を実行すると、複数のテンポラリファイルとmsiファイルが
　　　　作成されます。
　　　　任意のmsiファイルをご使用下さい。


4. インストール動作

　(1) インストールを行う場合、インストール先PCのレジストリをチェックし、
　　　Python2.4, 2.5, 2.6 の何れかが登録済みであるか判定します。
　　　何れも登録されていなければ、エラーメッセージを表示して、
　　　インストールを終了します。

　(2) Python2.4, 2.5, 2.6 の何れかが登録済みである場合

　　・Pythonインストールパスの各ディレクトリへ omniORBpy を
　　　インストールします。
　　・Pythonインストールパス\Lib\site-packages へ OpenRTM-aist を
　　　インストールします。
　　・Program Files フォルダへ ランタイムファイル、クラスリファレンス
　　　ファイル、examplesファイル をインストールします。

　(3) インストール完了時、IDLコンパイル起動バッチを実行し、
　　　Pythonインストールパス\Lib\site-packages\OpenRTM_aist\RTM_IDL の
　　　IDLファイル、Program Files フォルダ examples/SimpleService の
　　　IDLファイルをコンパイルします。

　(4) スタートボタンのプログラムメニューで
　　　・OpenRTM-aist -> Python -> documents より
　　　　日本語と英語のクラスリファレンスを選択出来ます。
　　　・OpenRTM-aist -> Python -> examples より
　　　　「Start Naming Service」と各example を選択出来ます。


5. アンインストール動作

　(1) アンインストールを行う場合、インストールファイルとは別に、
　　　Pythonスクリプト実行時に作成される.pyc ファイル、
　　　Program Files\OpenRTM-aist\1.0\examples\Python ディレクトリ、
　　　IDLコンパイルで作成された Python**\Lib\site-packages\OpenRTM_aist
　　　ディレクトリなどを削除します。

以上
