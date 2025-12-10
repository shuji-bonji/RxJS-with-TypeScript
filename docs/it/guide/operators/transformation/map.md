---
description: "L'operatore map applica una funzione a ciascun valore all'interno di un Observable per generare un nuovo valore. È un mezzo di trasformazione fondamentale ampiamente utilizzato per formattazione di form, elaborazione di risposte API e trasformazione dati. Spiega l'inferenza di tipo TypeScript, la combinazione con altri operatori e l'ottimizzazione delle prestazioni."
---

# map - Applica una funzione di trasformazione a ciascun valore

L'operatore `map` applica una funzione specificata a **ciascun valore** all'interno di un flusso, generando un nuovo valore trasformato.
È simile al metodo `Array.prototype.map` degli array, ma opera su **flussi asincroni**.


## 🔰 Sintassi di Base e Utilizzo

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3).pipe(
  map(value => value * 10)
).subscribe(console.log);
// Output: 10, 20, 30
```

Applica la funzione value => value * 10 a ciascun valore, generando nuovi valori.

[🌐 Documentazione Ufficiale RxJS - map](https://rxjs.dev/api/index/function/map)


## 💡 Pattern di Utilizzo Tipici
- Trasformazione delle risposte API (estrazione solo delle proprietà necessarie)
- Formattazione dei dati di input dei form
- Elaborazione di numeri e stringhe all'interno del flusso
- Estrazione solo dei dati necessari dagli eventi UI


## 🧠 Esempio di Codice Pratico (con UI)

Un esempio che raddoppia in tempo reale un numero inserito e lo visualizza.

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Creazione campo di input
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Inserisci un numero';
document.body.appendChild(input);

// Creazione campo di output
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Flusso di eventi di input
fromEvent(input, 'input').pipe(
  map(event => Number((event.target as HTMLInputElement).value)),
  map(value => value * 2)
).subscribe(result => {
  output.textContent = `Valore raddoppiato: ${result}`;
});
```

- Il valore di input viene raddoppiato in tempo reale e visualizzato nell'output.
- Applicando map consecutivamente, si realizza una catena di trasformazioni dati semplice.
