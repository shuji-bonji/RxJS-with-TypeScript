---
description: "L'operatore mergeMap trasforma ciascun valore in un nuovo Observable ed esegue in parallelo combinandoli in modo piatto. È utile quando si vogliono eseguire più richieste API in parallelo senza aspettare in coda o per gestire elaborazioni asincrone annidate."
---

# mergeMap - Trasforma ciascun valore in Observable e li unisce in parallelo

L'operatore `mergeMap` (alias `flatMap`) trasforma ciascun valore in un nuovo Observable e **li combina in modo piatto eseguendoli in parallelo**.
È molto utile quando si vogliono eseguire richieste immediatamente senza aspettare in coda o per elaborazioni asincrone annidate.

## 🔰 Sintassi di Base e Utilizzo

```ts
import { of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  mergeMap(value =>
    of(`${value} completato`).pipe(delay(1000))
  )
).subscribe(console.log);

// Esempio di output (ordine non garantito):
// A completato
// B completato
// C completato
```

- Genera un nuovo Observable per ciascun valore.
- Questi Observable vengono **eseguiti in parallelo** e i risultati vengono emessi in ordine non garantito.

[🌐 Documentazione Ufficiale RxJS - `mergeMap`](https://rxjs.dev/api/operators/mergeMap)

## 💡 Pattern di Utilizzo Tipici

- Inviare richieste API per ogni clic del pulsante
- Avviare l'upload di file per ogni evento di drop file
- Eseguire task asincroni in parallelo innescati da operazioni dell'utente

## 🧠 Esempio di Codice Pratico (con UI)

Un esempio che genera una richiesta asincrona (risposta dopo 2 secondi) ogni volta che si clicca il pulsante.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

// Creazione pulsante
const button = document.createElement('button');
button.textContent = 'Invia richiesta';
document.body.appendChild(button);

// Area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Eventi di clic
fromEvent(button, 'click').pipe(
  mergeMap((_, index) => {
    const requestId = index + 1;
    console.log(`Richiesta${requestId} iniziata`);
    return of(`Risposta${requestId}`).pipe(delay(2000));
  })
).subscribe((response) => {
  const div = document.createElement('div');
  div.textContent = `✅ ${response}`;
  output.appendChild(div);
});
```

- Una richiesta asincrona viene emessa immediatamente per ogni clic.
- **Ogni richiesta attende 2 secondi individualmente**, quindi i risultati non si allineano nell'ordine di arrivo.
- È un esempio ottimale per comprendere l'elaborazione parallela.
