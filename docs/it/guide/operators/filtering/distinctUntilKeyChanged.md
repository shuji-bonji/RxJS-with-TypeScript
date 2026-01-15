---
description: L'operatore distinctUntilKeyChanged si concentra su una proprietà specifica all'interno di uno stream di oggetti ed emette solo quando quel valore è diverso dal precedente. Salta efficientemente dati duplicati consecutivi ed è utile per rilevare cambiamenti di stato e ottimizzare aggiornamenti di liste.
titleTemplate: ':title'
---

# distinctUntilKeyChanged - Rileva Cambio Chiave

L'operatore `distinctUntilKeyChanged` si concentra su una chiave (proprietà) specifica di un oggetto ed emette solo quando quel valore è diverso dal precedente.
È utile per saltare efficientemente duplicati consecutivi.


## 🔰 Sintassi e Utilizzo Base

```ts
import { from } from 'rxjs';
import { distinctUntilKeyChanged } from 'rxjs';

const users = [
  { id: 1, name: 'Tanaka' },
  { id: 2, name: 'Tanaka' }, // Stesso nome, salta
  { id: 3, name: 'Sato' },
  { id: 4, name: 'Suzuki' },
  { id: 5, name: 'Suzuki' }, // Stesso nome, salta
  { id: 6, name: 'Tanaka' }
];

from(users).pipe(
  distinctUntilKeyChanged('name')
).subscribe(console.log);

// Output:
// { id: 1, name: 'Tanaka' }
// { id: 3, name: 'Sato' }
// { id: 4, name: 'Suzuki' }
// { id: 6, name: 'Tanaka' }
```

- Emette solo quando il valore della proprietà specificata `name` cambia.
- Altre proprietà (es. `id`) non vengono confrontate.

[🌐 Documentazione Ufficiale RxJS - `distinctUntilKeyChanged`](https://rxjs.dev/api/operators/distinctUntilKeyChanged)


## 💡 Pattern di Utilizzo Tipici

- Aggiorna visualizzazione lista solo quando una proprietà specifica cambia
- Rileva solo cambiamenti in attributi specifici negli stream di eventi
- Controlla rimozione duplicati su base chiave


## 🧠 Esempio di Codice Pratico (con UI)

Inserisci un nome nella casella di testo e premi Invio per registrarlo.
**Se lo stesso nome viene inserito consecutivamente, viene ignorato**, e viene aggiunto alla lista solo quando viene inserito un nome diverso.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, scan, distinctUntilKeyChanged } from 'rxjs';

// Crea area output
const output = document.createElement('div');
document.body.appendChild(output);

const title = document.createElement('h3');
title.textContent = 'Esempio Pratico distinctUntilKeyChanged';
output.appendChild(title);

// Form input
const input = document.createElement('input');
input.placeholder = 'Inserisci nome e premi Invio';
document.body.appendChild(input);

// Stream eventi input
fromEvent<KeyboardEvent>(input, 'keydown').pipe(
  filter((e) => e.key === 'Enter'),
  map(() => input.value.trim()),
  filter((name) => name.length > 0),
  scan((_, name, index) => ({ id: index + 1, name }), { id: 0, name: '' }),
  distinctUntilKeyChanged('name')
).subscribe((user) => {
  const item = document.createElement('div');
  item.textContent = `Input utente: ID=${user.id}, Nome=${user.name}`;
  output.appendChild(item);
});
```

- Se lo stesso nome viene inserito consecutivamente, viene saltato.
- Viene visualizzato solo quando viene inserito un nuovo nome.
