---
description: L'operatore distinctUntilChanged consente un'elaborazione dati efficiente saltando valori consecutivi uguali al precedente ed emettendo solo i valori che sono cambiati.
titleTemplate: ':title | RxJS'
---

# distinctUntilChanged - Ignora duplicati

L'operatore `distinctUntilChanged` rimuove i duplicati quando lo stesso valore viene emesso consecutivamente, ed emette il nuovo valore solo se è diverso dal precedente.


## 🔰 Sintassi e Utilizzo Base

```ts
import { from } from 'rxjs';
import { distinctUntilChanged } from 'rxjs';

const numbers$ = from([1, 1, 2, 2, 3, 1, 2, 3]);

numbers$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Output: 1, 2, 3, 1, 2, 3
```

- Se il valore è uguale al precedente, viene ignorato.
- Non è un processo batch come `Array.prototype.filter`, ma piuttosto una **decisione sequenziale**.

[🌐 Documentazione Ufficiale RxJS - `distinctUntilChanged`](https://rxjs.dev/api/operators/distinctUntilChanged)


## 💡 Pattern di Utilizzo Tipici

- Rilevamento input form per prevenire richieste inutili per valori di input consecutivi uguali
- Rilevamento cambiamenti in sensori e stream di eventi
- Prevenire ridisegni UI non necessari nella gestione dello stato


## 🧠 Esempio di Codice Pratico (con UI)

Simulazione di invio di una richiesta API in una casella di ricerca **solo se la stringa inserita è diversa dalla precedente**.

```ts
import { fromEvent } from 'rxjs';
import { map, distinctUntilChanged } from 'rxjs';

// Crea area output
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Inserisci parole chiave di ricerca';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Stream input
fromEvent(searchInput, 'input')
  .pipe(
    distinctUntilChanged(),
    map((event) => (event.target as HTMLInputElement).value.trim())
  )
  .subscribe((keyword) => {
    resultArea.textContent = `Esegui con valore di ricerca: ${keyword}`;
  });

```

- Se il testo di input non cambia, non viene fatta la richiesta.
- Questo può essere usato per elaborazione ricerca efficiente e ottimizzazione comunicazione API.
