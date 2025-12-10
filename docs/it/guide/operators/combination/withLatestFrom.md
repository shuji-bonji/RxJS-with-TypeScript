---
description: "L'operatore withLatestFrom combina l'ultimo valore da un altro stream ogni volta che l'Observable principale emette: Ideale per validazione form e sincronizzazione stato"
titleTemplate: ':title'
---

# withLatestFrom - Combina l'Ultimo Valore contro l'Emissione dello Stream Principale

L'operatore `withLatestFrom` **ogni volta che un valore nello stream principale viene emesso**,
combina l'**ultimo valore** da un altro stream e lo emette.


## 🔰 Sintassi e Utilizzo Base

```ts
import { interval, fromEvent } from 'rxjs';
import { withLatestFrom, map, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');
const timer$ = interval(1000);

clicks$
  .pipe(
    withLatestFrom(timer$),
    map(([click, timerValue]) => `Contatore al momento del click: ${timerValue}`)
  )
  .subscribe(console.log);

// Output:
// Contatore al momento del click: 1
// Contatore al momento del click: 2
// Contatore al momento del click: 2
// Contatore al momento del click: 5

```

- L'Observable principale (in questo caso, i click) agisce come trigger,
- L'**ultimo valore** dell'Observable secondario (in questo caso, il contatore) viene combinato ed emesso ogni volta.

[🌐 Documentazione Ufficiale RxJS - `withLatestFrom`](https://rxjs.dev/api/index/function/withLatestFrom)


## 💡 Pattern di Utilizzo Tipici

- **Ottenere l'ultimo stato al momento dell'azione utente**
- **Riferire dati in cache al momento della richiesta**
- **Binding dati attivato da eventi**


## 🧠 Esempio di Codice Pratico (con UI)

Esempio di recupero e visualizzazione dell'ultimo valore di un campo input ogni 2 secondi.

```ts
import { fromEvent, interval } from 'rxjs';
import { map, startWith, withLatestFrom } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'withLatestFrom: Ottieni Ultimo Input Ogni 2 Secondi:';
document.body.appendChild(title);

// Crea campo input
const nameInput = document.createElement('input');
nameInput.placeholder = 'Inserisci nome';
document.body.appendChild(nameInput);

// Crea area output
const output = document.createElement('div');
document.body.appendChild(output);

// Observable Input
const name$ = fromEvent(nameInput, 'input').pipe(
  map((e) => (e.target as HTMLInputElement).value),
  startWith('') // Inizia con stringa vuota
);

// Timer (si attiva ogni 2 secondi)
const timer$ = interval(2000);

// Ottieni l'ultimo valore di input ogni volta che il timer si attiva
timer$.pipe(withLatestFrom(name$)).subscribe(([_, name]) => {
  const item = document.createElement('div');
  item.textContent = `Recupero 2 secondi: Nome: ${name}`;
  output.prepend(item);
});

```

- Mentre l'utente continua a digitare,
- **L'ultimo input viene recuperato e visualizzato** ogni 2 secondi.
