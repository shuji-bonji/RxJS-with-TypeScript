---
description: "concatMap è un operatore di trasformazione che elabora ciascun Observable in sequenza, aspettando che il precedente sia completato prima di procedere. È ideale per scenari in cui l'ordine di esecuzione è importante, come l'esecuzione seriale di chiamate API o la garanzia dell'ordine di upload dei file. Realizza concatenazioni di elaborazioni asincrone type-safe con l'inferenza di tipo TypeScript e spiega anche le differenze con mergeMap e switchMap."
---

# concatMap - Esegue ciascun Observable in sequenza

L'operatore `concatMap` trasforma ciascun valore del flusso di input in un Observable e **li esegue e combina in sequenza**.
La caratteristica è che **non avvia l'Observable successivo finché quello precedente non è completato**.

## 🔰 Sintassi di Base e Utilizzo

```ts
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  concatMap(value =>
    of(`${value} completato`).pipe(delay(1000))
  )
).subscribe(console.log);

// Output (in ordine):
// A completato
// B completato
// C completato
```
- Trasforma ciascun valore in un Observable.
- L'Observable successivo viene eseguito dopo che quello precedente è completato.

[🌐 Documentazione Ufficiale RxJS - concatMap](https://rxjs.dev/api/index/function/concatMap)

## 💡 Pattern di Utilizzo Tipici
- Esecuzione di richieste API in cui l'ordine è importante
- Elaborazione task basata su coda
- Controllo di animazioni o UI step-by-step
- Elaborazione invio messaggi in cui l'ordine di invio è importante


## 🧠 Esempio di Codice Pratico (con UI)

Un esempio in cui ogni clic del pulsante genera una richiesta, e le richieste vengono sempre elaborate in ordine.

```ts
import { fromEvent, of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

// Creazione pulsante
const button = document.createElement('button');
button.textContent = 'Invia richiesta';
document.body.appendChild(button);

// Area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Eventi di clic
fromEvent(button, 'click')
  .pipe(
    concatMap((_, index) => {
      const requestId = index + 1;
      console.log(`Richiesta${requestId} iniziata`);
      return of(`Risposta${requestId}`).pipe(delay(2000));
    })
  )
  .subscribe((response) => {
    const div = document.createElement('div');
    div.textContent = `✅ ${response}`;
    output.appendChild(div);
  });

```

- Ogni richiesta viene sempre inviata e completata in ordine.
- La richiesta successiva viene emessa dopo che quella precedente è completata.
