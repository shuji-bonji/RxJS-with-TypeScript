---
description: "L'operatore first ottiene il primo valore da uno stream o il primo valore che soddisfa una condizione specificata, completando poi lo stream. Utile per elaborare solo il primo evento raggiunto o ottenere dati iniziali."
---

# first - Ottieni Primo Valore

L'operatore `first` ottiene solo il **primo valore** o il **primo valore che soddisfa la condizione** da uno stream e completa lo stream.


## 🔰 Sintassi di base e utilizzo

```ts
import { from } from 'rxjs';
import { first } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Ottiene solo il primo valore
numbers$.pipe(
  first()
).subscribe(console.log);

// Ottiene solo il primo valore che soddisfa la condizione
numbers$.pipe(
  first(n => n > 3)
).subscribe(console.log);

// Output:
// 1
// 4
```

- `first()` ottiene il primo valore emesso e completa.
- Passando una condizione, ottiene il **primo valore che soddisfa la condizione**.
- Se non esiste un valore che soddisfa la condizione, genera un errore.

[🌐 Documentazione ufficiale RxJS - `first`](https://rxjs.dev/api/operators/first)


## 💡 Pattern di utilizzo tipici

- Elaborare solo il primo evento raggiunto
- Rilevare il primo dato che soddisfa una condizione (es: punteggio sopra 5)
- Adottare solo il primo dato arrivato prima del timeout o cancellazione


## 🧠 Esempio di codice pratico (con UI)

Anche cliccando il pulsante più volte, elabora **solo il primo clic**.

```ts
import { fromEvent } from 'rxjs';
import { first } from 'rxjs';

const title = document.createElement('div');
title.innerHTML = '<h3>Esempio pratico di first:</h3>';
document.body.appendChild(title);

// Crea pulsante
const button = document.createElement('button');
button.textContent = 'Clicca qui (reagisce solo la prima volta)';
document.body.appendChild(button);

// Crea area di output
let count = 0;
const output = document.createElement('div');
document.body.appendChild(output);
// Stream di clic sul pulsante
fromEvent(button, 'click')
  .pipe(first())
  .subscribe(() => {
    const message = document.createElement('div');
    count++;
    message.textContent = `Rilevato primo clic! ${count}`;
    output.appendChild(message);
  });
```

- Riceve solo il primo evento di clic, i successivi vengono ignorati.
- Lo stream completa automaticamente dopo il primo clic con `complete`.