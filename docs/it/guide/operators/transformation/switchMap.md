---
description: "switchMap è un operatore di trasformazione che annulla l'Observable precedente e passa a quello più recente. È ideale per casi d'uso come ricerca live, cambio di navigazione e salvataggio automatico, realizzando elaborazioni asincrone sicure con l'inferenza di tipo TypeScript. Spiega anche le differenze con mergeMap e concatMap."
---

# switchMap - Passa all'ultimo

L'operatore `switchMap` genera un nuovo Observable per ciascun valore del flusso di input e **annulla l'Observable precedente passando solo a quello più recente**.
È ideale per casi come i form di ricerca, dove si vuole rendere valido solo l'input più recente.

## 🔰 Sintassi di Base e Utilizzo

```ts
import { of } from 'rxjs';
import { delay, switchMap } from 'rxjs';

of('A', 'B', 'C').pipe(
  switchMap(value =>
    of(`${value} completato`).pipe(delay(1000))
  )
).subscribe(console.log);

// Esempio di output:
// C completato
```

- Crea un nuovo Observable per ciascun valore.
- Tuttavia, **nel momento in cui arriva un nuovo valore, l'Observable precedente viene annullato**.
- Alla fine, viene emesso solo `C`.

[🌐 Documentazione Ufficiale RxJS - `switchMap`](https://rxjs.dev/api/operators/switchMap)

## 💡 Pattern di Utilizzo Tipici

- Autocompletamento dei form di input
- Funzionalità di ricerca live (valido solo l'input più recente)
- Caricamento risorse durante il cambio di navigazione o routing
- Quando si vuole passare all'azione più recente dell'utente

## 🧠 Esempio di Codice Pratico (con UI)

Quando si inseriscono caratteri nella casella di ricerca, viene inviata immediatamente una richiesta API e vengono visualizzati i risultati **solo dell'ultimo input**.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

// Creazione campo di input
const searchInput = document.createElement('input');
searchInput.placeholder = 'Cerca per nome utente';
document.body.appendChild(searchInput);

// Area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Elaborazione eventi di input
fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  map(event => (event.target as HTMLInputElement).value.trim()),
  switchMap(term => {
    if (term === '') {
      return of([]);
    }
    return ajax.getJSON(`https://jsonplaceholder.typicode.com/users?username_like=${term}`);
  })
).subscribe(users => {
  output.innerHTML = '';

  (users as any[]).forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.username;
    output.appendChild(div);
  });
});
```

- Ogni volta che l'input cambia, la richiesta precedente viene annullata.
- Vengono visualizzati solo gli utenti che corrispondono alla parola di ricerca più recente.
