---
description: "L'operatore last estrae l'ultimo valore o l'ultimo valore che corrisponde a una condizione al completamento dello stream. Differenze con first(), impostazione valori predefiniti, gestione EmptyError, implementazione type-safe TypeScript e differenze con takeLast()."
---

# last - Ottiene l'ultimo valore o l'ultimo valore che soddisfa la condizione

L'operatore `last` ottiene l'**ultimo valore** o l'**ultimo valore che soddisfa la condizione** da uno stream e completa lo stream.


## 🔰 Sintassi di base e utilizzo

```ts
import { from } from 'rxjs';
import { last } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Ottiene solo l'ultimo valore
numbers$.pipe(
  last()
).subscribe(console.log);

// Ottiene solo l'ultimo valore che soddisfa la condizione
numbers$.pipe(
  last(n => n < 5)
).subscribe(console.log);

// Output:
// 5
// 4
```

- `last()` emette l'**ultimo valore emesso** al completamento dello stream.
- Passando una condizione, è possibile ottenere solo l'**ultimo valore che soddisfa la condizione**.
- Se non esiste un valore che soddisfa la condizione, si verifica un errore.

[🌐 Documentazione ufficiale RxJS - `last`](https://rxjs.dev/api/operators/last)


## 💡 Pattern di utilizzo tipici

- Ottenere l'ultimo elemento di dati filtrati
- Ottenere lo stato più recente al completamento dello stream
- Estrarre l'ultima operazione importante da sessione o log operazioni


## 🧠 Esempio di codice pratico (con UI)

Tra più numeri inseriti in 5 volte, ottiene e visualizza l'ultimo valore inferiore a 5.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, take, last } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Esempio pratico di last:</h3>';
document.body.appendChild(output);

// Crea campo di input
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Inserisci numero e premi Enter';
document.body.appendChild(input);

// Stream di eventi di input
fromEvent<KeyboardEvent>(input, 'keydown')
  .pipe(
    filter((e) => e.key === 'Enter'),
    map(() => parseInt(input.value, 10)),
    take(5), // Prende solo i primi 5 e poi completa
    filter((n) => !isNaN(n) && n < 5), // Fa passare solo valori inferiori a 5
    last() // Ottiene l'ultimo valore inferiore a 5
  )
  .subscribe({
    next: (value) => {
      const item = document.createElement('div');
      item.textContent = `Ultimo valore inferiore a 5: ${value}`;
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
1. Inserisci numeri 5 volte e premi Enter
2. Seleziona solo "valori inferiori a 5" tra i numeri inseriti
3. Visualizza solo l'ultimo numero inferiore a 5 inserito
4. Lo stream completa naturalmente e termina