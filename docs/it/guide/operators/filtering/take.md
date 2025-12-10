---
description: "L'operatore take ottiene solo il numero specificato di primi valori da uno stream Observable e ignora i valori successivi completando automaticamente lo stream. Utile per ottenere solo i primi dati."
---

# take - Ottiene solo il numero specificato di primi valori

L'operatore `take` ottiene solo il **numero specificato** di valori da uno stream e ignora i valori successivi.
Dopo il completamento, lo stream esegue automaticamente `complete`.

## 🔰 Sintassi di base e utilizzo

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2
```

- Sottoscrive ottenendo solo i primi 3 valori.
- Dopo aver ottenuto 3 valori, l'Observable esegue automaticamente `complete`.

[🌐 Documentazione ufficiale RxJS - `take`](https://rxjs.dev/api/operators/take)

## 💡 Pattern di utilizzo tipici

- Visualizzare o registrare solo i primi valori in UI o log
- Sottoscrizione temporanea per estrarre solo la prima risposta
- Acquisizione limitata di dati di test o demo

## 🧠 Esempio di codice pratico (con UI)

Ottiene e visualizza solo i primi 5 valori da numeri emessi ogni secondo.

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio pratico di take:</h3>';
document.body.appendChild(output);

// Emette valore ogni secondo
const source$ = interval(1000);

// Ottiene solo i primi 5 valori
source$.pipe(take(5)).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `Valore: ${value}`;
    output.appendChild(item);
  },
  complete: () => {
    const complete = document.createElement('div');
    complete.textContent = 'Completato';
    complete.style.fontWeight = 'bold';
    output.appendChild(complete);
  },
});

```

- I primi 5 valori (`0`, `1`, `2`, `3`, `4`) vengono visualizzati in ordine
- Successivamente viene visualizzato il messaggio "Completato"