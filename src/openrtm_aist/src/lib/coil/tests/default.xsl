<!--

 @file default.xsl
 @brief XSL file for CppUnit test result page
 @date $Date: 2006-11-27 07:34:28 $
 @author Noriaki Ando <n-ando@aist.go.jp>
 Copyright (C) 2006 Noriaki Ando

 $Id: default.xsl 826 2008-08-26 08:13:39Z n-ando $

-->
<?xml version="1.0" encoding="shift_jis"?>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/TR/WD-xsl" version="1.0" xml:lang="ja">

<xsl:template match="/">
  <HTML>
  <HEAD>
  <TITLE>UnitTestの結果</TITLE>
  <link rel="stylesheet" type="text/css" href="default.css" />
  <meta http-equiv="Content-Type" content="text/html; charset=Shift_JIS" />  
  </HEAD>
  <BODY>
  <H1>CppUnit テスト結果</H1>
  <H2>テスト統計情報</H2>
  <P>
  成功:   <xsl:value-of select="TestRun/Statistics/Tests" />
  失敗:   <xsl:value-of select="TestRun/Statistics/FailuresTotal" />
  エラー: <xsl:value-of select="TestRun/Statistics/Errors" />
  </P>
  <H2>テスト一覧</H2>
  <xsl:apply-templates select="TestRun" />

  </BODY>
  </HTML>
</xsl:template>

<xsl:template match="TestRun">
  <TABLE>
  <TR>
  <TH>ID</TH>
  <TH>失敗テスト名</TH>
  <TH>状態</TH>
  <TH>ファイル(行)</TH>
  <TH>失敗の内容</TH>
  </TR>
  <xsl:for-each select="FailedTests/FailedTest">
  <TR>
  <TD><xsl:value-of select="@id" /></TD>
  <TD><xsl:value-of select="Name" /></TD>
  <TD><xsl:value-of select="FailureType" /></TD>
  <TD><xsl:value-of select="Location/File" />
  (<xsl:value-of select="Location/Line" />)</TD>
  <TD>
  <xsl:value-of select="./text()" />
  <xsl:value-of select="Message" />
  </TD>
  </TR>
  </xsl:for-each>
  <TR>
  <TH>ID</TH>
  <TH>成功テスト名</TH>
  <TH>状態</TH>
  <TH>ファイル(行)</TH>
  <TH>失敗の内容</TH>
  </TR>
  <xsl:for-each select="SuccessfulTests/Test">
  <TR>
  <TD><xsl:value-of select="@id" /></TD>
  <TD><xsl:value-of select="Name" /></TD>
  <TD>Success</TD>
  <TD> ---- </TD>
  <TD> ---- </TD>
  </TR>
  </xsl:for-each>
  </TABLE>
</xsl:template>

</xsl:stylesheet>

