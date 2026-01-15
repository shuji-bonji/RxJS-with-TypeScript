---
description: "L'operatore exhaustMap è un operatore di trasformazione che ignora nuovi input finché l'Observable in corso di elaborazione non è completato. È efficace in situazioni in cui si vuole limitare l'esecuzione parallela, come la prevenzione di clic multipli su pulsanti di invio form o la prevenzione di invii duplicati di richieste API."
---

# exhaustMap - Ignora input

L'operatore `exhaustMap` **ignora nuovi input** finché l'Observable in corso di elaborazione non è completato.
È ideale per prevenire clic duplicati o invii multipli di richieste.

## 🔰 Sintassi di Base e Utilizzo

```ts
import { fromEvent, of } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(exhaustMap(() => of('Richiesta completata').pipe(delay(1000))))
  .subscribe(console.log);

// Esempio di output:
// (Solo il primo clic emette "Richiesta completata" dopo 1 secondo)

```

- Finché la richiesta in corso non è completata, gli input successivi vengono ignorati.

[🌐 Documentazione Ufficiale RxJS - `exhaustMap`](https://rxjs.dev/api/operators/exhaustMap)

## 💡 Pattern di Utilizzo Tipici

- Prevenzione di clic multipli su pulsanti di invio form
- Prevenzione di richieste doppie (specialmente in login, pagamenti, ecc.)
- Controllo visualizzazione singola di modal o dialog

## 🧠 Esempio di Codice Pratico (con UI)

Quando si clicca il pulsante di invio, inizia l'elaborazione di invio.
**Anche se si clicca più volte durante l'invio, viene ignorato** e non accetta l'invio successivo finché la prima elaborazione non è completata.

```ts
import { fromEvent } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Creazione pulsante
const submitButton = document.createElement('button');
submitButton.textContent = 'Invia';
document.body.appendChild(submitButton);

// Creazione area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Elaborazione invio
fromEvent(submitButton, 'click')
  .pipe(
    exhaustMap(() => {
      output.textContent = 'Invio in corso...';
      return ajax
        .post('https://jsonplaceholder.typicode.com/posts', {
          title: 'foo',
          body: 'bar',
          userId: 1,
        })
        .pipe(delay(2000)); // Simula ritardo di invio di 2 secondi
    })
  )
  .subscribe({
    next: (response) => {
      output.textContent = 'Invio riuscito!';
      console.log('Invio riuscito:', response);
    },
    error: (error) => {
      output.textContent = 'Errore di invio';
      console.error('Errore di invio:', error);
    },
  });

```

- Anche se ci sono altri clic durante il clic del pulsante, vengono ignorati.
- Dopo 2 secondi viene visualizzato "Invio riuscito!" o "Errore di invio".
