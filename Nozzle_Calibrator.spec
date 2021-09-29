# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['Nozzle_Calibrator.py'],
             pathex=['D:\\Workroom\\3D_Michelan\\Michelan_Jig'],
             binaries=[],
             datas=[('./*.ui','./'),
                ('./Images/*.jpg','./Images'),
                ('./*.wav','./'),
                ('./*.ico','./')
             ],
             hiddenimports=[
                'PyQt5.QtGui',
                'PyQt5.uic',
                'matplotlib.pyplot'
             ],
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          [],
          exclude_binaries=True,
          name='Nozzle_Calibrator',
          debug=True,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          console=True , icon='icon_final.ico')
coll = COLLECT(exe,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=False,
               upx=True,
               upx_exclude=[],
               name='Nozzle_Calibrator')
