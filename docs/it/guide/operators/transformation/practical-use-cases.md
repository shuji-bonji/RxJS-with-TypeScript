---
description: Questa sezione copre i casi d'uso pratici degli operatori di trasformazione RxJS (map, mergeMap, switchMap, concatMap, scan, buffer, ecc.). Pattern pratici per elaborare e trasformare flessibilmente i dati come elaborazione input utente, formattazione risposte API, richieste annidate, aggregazione dati, suddivisione stream e elaborazione batch sono introdotti con esempi di codice TypeScript. Imparerai i pattern di trasformazione frequentemente usati nella pratica.
---

# Pattern di Trasformazione Pratici

Gli operatori di trasformazione sono uno dei gruppi di operatori più frequentemente usati in RxJS.
Giocano un ruolo essenziale nella programmazione reattiva per elaborare e trasformare flessibilmente i dati.

In questa sezione, organizzeremo i pattern di utilizzo degli operatori di trasformazione presentando esempi pratici tipici.

## 💬 Pattern di Utilizzo Tipici

| Pattern | Operatori Rappresentativi | Descrizione |
|:---|:---|:---|
| Conversione semplice di valori | `map` | Applica funzione di conversione a ogni valore |
| Accumulo e aggregazione | `scan`, `reduce` | Accumulo sequenziale di valori |
| Elaborazione asincrona annidata | `mergeMap`, `switchMap`, `concatMap`, `exhaustMap` | Genera e combina Observable |
| Elaborazione batch e raggruppamento | `bufferTime`, `bufferCount`, `windowTime` | Elaborazione collettiva e gestione partizionamento |
| Estrazione proprietà | `pluck` | Estrai campi specifici da oggetti |

## Validazione e Conversione Input Utente

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

// Campo di input
const emailInput = document.createElement('input');
const emailStatus = document.createElement('p');
document.body.appendChild(emailInput);
document.body.appendChild(emailStatus);

// Funzione di validazione email
function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

// Elaborazione input
fromEvent(emailInput, 'input')
  .pipe(
    debounceTime(400),
    map((event) => (event.target as HTMLInputElement).value.trim()),
    distinctUntilChanged(),
    map((email) => {
      if (!email) {
        return {
          isValid: false,
          message: 'Inserisci un indirizzo email',
          value: email,
        };
      }

      if (!isValidEmail(email)) {
        return {
          isValid: false,
          message: 'Inserisci un indirizzo email valido',
          value: email,
        };
      }

      return {
        isValid: true,
        message: "L'indirizzo email è valido",
        value: email,
      };
    })
  )
  .subscribe((result) => {
    if (result.isValid) {
      emailStatus.textContent = '✓ ' + result.message;
      emailStatus.className = 'valid';
    } else {
      emailStatus.textContent = '✗ ' + result.message;
      emailStatus.className = 'invalid';
    }
  });
```

## Conversione e Aggregazione Array di Oggetti

```ts
import { from } from 'rxjs';
import { map, toArray } from 'rxjs';

// Dati di vendita
const sales = [
  { product: 'Laptop', price: 120000, quantity: 3 },
  { product: 'Tablet', price: 45000, quantity: 7 },
  { product: 'Smartphone', price: 85000, quantity: 4 },
  { product: 'Mouse', price: 3500, quantity: 12 },
  { product: 'Tastiera', price: 6500, quantity: 8 },
];

// Conversione e aggregazione dati
from(sales)
  .pipe(
    // Calcola importo totale per ogni prodotto
    map((item) => ({
      product: item.product,
      price: item.price,
      quantity: item.quantity,
      total: item.price * item.quantity,
    })),
    // Aggiungi prezzo con IVA inclusa
    map((item) => ({
      ...item,
      totalWithTax: Math.round(item.total * 1.1),
    })),
    // Riconverti in array
    toArray(),
    // Calcola totale complessivo
    map((items) => {
      const grandTotal = items.reduce((sum, item) => sum + item.total, 0);
      const grandTotalWithTax = items.reduce(
        (sum, item) => sum + item.totalWithTax,
        0
      );
      return {
        items,
        grandTotal,
        grandTotalWithTax,
      };
    })
  )
  .subscribe((result) => {
    console.log('Dettagli prodotto:', result.items);
    console.log('Totale complessivo (IVA esclusa):', result.grandTotal);
    console.log('Totale complessivo (IVA inclusa):', result.grandTotalWithTax);
  });
// Output:
// Dettagli prodotto: (5) [{…}, {…}, {…}, {…}, {…}]
// Totale complessivo (IVA esclusa): 1109000
// Totale complessivo (IVA inclusa): 1219900
```

## Normalizzazione Dati JSON

```ts
import { ajax } from 'rxjs/ajax';
import { map } from 'rxjs';

const resultBox = document.createElement('div');
resultBox.id = 'normalized-results';
document.body.appendChild(resultBox);

ajax
  .getJSON<any[]>('https://jsonplaceholder.typicode.com/users')
  .pipe(
    map((users) => {
      // Converti in oggetto con ID come chiave
      const normalizedUsers: Record<number, any> = {};
      const userIds: number[] = [];

      users.forEach((user) => {
        normalizedUsers[user.id] = {
          ...user,
          // Appiattisci oggetti annidati
          companyName: user.company.name,
          city: user.address.city,
          street: user.address.street,
          // Rimuovi annidamento non necessario
          company: undefined,
          address: undefined,
        };
        userIds.push(user.id);
      });

      return {
        entities: normalizedUsers,
        ids: userIds,
      };
    })
  )
  .subscribe((result) => {
    const title = document.createElement('h3');
    title.textContent = 'Dati Utente Normalizzati';
    resultBox.appendChild(title);

    result.ids.forEach((id) => {
      const user = result.entities[id];
      const div = document.createElement('div');
      div.innerHTML = `
      <strong>${user.name}</strong><br>
      Username: @${user.username}<br>
      Email: ${user.email}<br>
      Azienda: ${user.companyName}<br>
      Indirizzo: ${user.city}, ${user.street}<br><br>
    `;
      resultBox.appendChild(div);
    });

    // Accesso rapido all'utente per ID specifico
    console.log('Utente ID 3:', result.entities[3]);
  });

```

## Combinazione di Trasformazioni Multiple

Nelle applicazioni reali, è comune usare una combinazione di più operatori di trasformazione.

```ts
import { fromEvent, timer } from 'rxjs';
import {
  switchMap,
  map,
  tap,
  debounceTime,
  takeUntil,
  distinctUntilChanged,
} from 'rxjs';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
  company: {
    name: string;
  };
};

// Input di ricerca
const searchInput = document.createElement('input');
const resultsContainer = document.createElement('p');
const loadingIndicator = document.createElement('p');

document.body.append(searchInput);
document.body.append(resultsContainer);
document.body.append(loadingIndicator);

// Elaborazione ricerca
fromEvent(searchInput, 'input')
  .pipe(
    // Ottieni valore di input
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Attendi 300ms
    debounceTime(300),
    // Ignora stesso valore
    distinctUntilChanged(),
    // Mostra caricamento
    tap(() => {
      loadingIndicator.style.display = 'block';
      resultsContainer.innerHTML = '';
    }),
    // Richiesta API (cancella richiesta precedente)
    switchMap((term) => {
      // Input vuoto restituisce nessun risultato
      if (term === '') {
        return [];
      }

      // Gestione timeout (5 secondi)
      const timeout$ = timer(5000).pipe(
        tap(() => console.warn('Timeout risposta API')),
        map(() => [{ error: 'Timeout' }])
      );

      // Chiamata API
      const response$ = ajax
        .getJSON(
          `https://jsonplaceholder.typicode.com/users?username_like=${term}`
        )
        .pipe(
          // Elabora risultati
          map((users) =>
            (users as User[]).map((user) => ({
              id: user.id,
              name: user.name,
              username: user.username,
              email: user.email,
              company: user.company.name,
            }))
          ),
          // Completa prima del timeout
          takeUntil(timeout$)
        );

      return response$;
    }),
    // Nascondi caricamento
    tap(() => {
      loadingIndicator.style.display = 'none';
    })
  )
  .subscribe((result) => {
    loadingIndicator.style.display = 'none';

    if (Array.isArray(result)) {
      if (result.length === 0) {
        resultsContainer.innerHTML =
          '<div class="no-results">Nessun utente trovato</div>';
      } else {
        resultsContainer.innerHTML = result
          .map(
            (user) => `
          <div class="user-card">
            <h3>${user.name}</h3>
            <p>@${user.username}</p>
            <p>${user.email}</p>
            <p>Azienda: ${user.company}</p>
          </div>
        `
          )
          .join('');
      }
    } else {
      resultsContainer.innerHTML = `<div class="error">⚠️ ${result}</div>`;
    }
  });

```

## 🧠 Riepilogo

- Per conversioni semplici, usa `map`
- Per gestire operazioni asincrone, usa `mergeMap`, `switchMap`, `concatMap`, `exhaustMap`
- Per elaborazione batch, usa `bufferTime`, `bufferCount`
- Per estrazione proprietà, usa `pluck`
- **Nelle app reali, combinare questi operatori è la norma**

Una volta padroneggiati gli operatori di trasformazione, puoi gestire flussi di dati asincroni complessi
in modo intuitivo e dichiarativo!
