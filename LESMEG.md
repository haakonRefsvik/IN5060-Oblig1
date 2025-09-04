## Kjøring

- Lag en virtelt miljø med python-versjon 3.10, og kjør den

[link til python 3.10](https://www.python.org/ftp/python/3.10.9/python-3.10.9-macos11.pkg)

```
python3.10 -m venv env
source env/bin/activate
```

- Last ned alle tilhørende pakker i det virtuelle-miljøet

```
pip install -r requirements.txt
pip install -e .
```

- I det virtuelle miljøet, gå i **examples** mappen og prøv å kjør **common_examples.py** ved å skrive.

```
python3 common_examples.py
```

- Matplotlib kan bruke litt tid her.
