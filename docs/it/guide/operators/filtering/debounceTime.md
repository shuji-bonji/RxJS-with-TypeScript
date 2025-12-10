---
description: L'operatore debounceTime emette l'ultimo valore quando nessun nuovo valore è stato ricevuto per un tempo specificato dopo l'emissione di eventi consecutivi. È ideale per ottimizzare input frequenti come la digitazione in caselle di ricerca o eventi di ridimensionamento finestra.
titleTemplate: ':title'
---

# debounceTime - Rallenta l'Emissione degli Eventi ed Emetti il Valore Dopo Aver Atteso un Certo Tempo

L'operatore `debounceTime` emette l'ultimo valore dopo che un valore è stato emesso nello stream se nessun nuovo valore è stato emesso per il tempo specificato.
È molto comunemente usato in situazioni dove eventi frequenti devono essere soppressi, come caselle di ricerca con input utente.

## 🔰 Sintassi e Utilizzo Base

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const searchBox = document.createElement('input');
document.body.appendChild(searchBox);

fromEvent(searchBox, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value),
    debounceTime(300)
  )
  .subscribe(console.log);
```

- Se nessun ulteriore input viene ricevuto entro 300ms dopo un evento input, il valore viene emesso.
- Questo ha l'effetto di consolidare eventi che si verificano consecutivamente in un breve periodo di tempo.

[🌐 Documentazione Ufficiale RxJS - `debounceTime`](https://rxjs.dev/api/operators/debounceTime)

## 💡 Pattern di Utilizzo Tipici

- Invia richiesta dopo che l'utente finisce di digitare nella casella di ricerca
- Ottieni dimensione finale per evento di ridimensionamento finestra
- Ottieni posizione finale per evento scroll

## 🧠 Esempio di Codice Pratico (con UI)

Quando un carattere viene inserito nella casella di ricerca, un messaggio di inizio ricerca viene visualizzato quando l'input si ferma per 300 ms.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

// Crea area output
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Inserisci parola di ricerca';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Stream input
fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300)
).subscribe(value => {
  resultArea.textContent = `Ricerca avviata per "${value}"`;
});
```

- Nessuna risposta immediata durante l'input
- Si fermerà l'input e inizierà la ricerca con l'ultimo valore di input 300ms dopo
