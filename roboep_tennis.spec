# -*- mode: python -*-

block_cipher = None


a = Analysis(['roboep_tennis.py'],
             pathex=None,
             binaries=None,
             datas=None,
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,
          name='roboep_tennis',
          debug=False,
          strip=False,
          upx=True,
          console=False,
          icon='tennis.ico')
