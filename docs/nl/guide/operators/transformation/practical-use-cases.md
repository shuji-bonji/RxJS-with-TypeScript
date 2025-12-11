---
description: Deze sectie behandelt praktische use cases van RxJS transformatieoperators (map, mergeMap, switchMap, concatMap, scan, buffer, etc.). Praktische patronen voor het flexibel verwerken en transformeren van data zoals gebruikersinvoerverwerking, API-responsopmaak, geneste verzoeken, data-aggregatie, streamsplitsing en batchverwerking worden gepresenteerd met TypeScript codevoorbeelden. U leert transformatiepatronen die vaak in de praktijk worden gebruikt.
---

# Praktische transformatiepatronen

Transformatieoperators zijn een van de meest gebruikte groepen operators in RxJS.
Ze spelen een essentiële rol in reactief programmeren om data flexibel te verwerken en te transformeren.

In deze sectie organiseren we de patronen van gebruik van de transformatieoperators door typische praktische voorbeelden te presenteren.

## 💬 Typische gebruikspatronen

| Patroon | Representatieve operators | Beschrijving |
|:---|:---|:---|
| Eenvoudige waardeconversie | `map` | Pas conversiefunctie toe op elke waarde |
| Accumulatie en aggregatie | `scan`, `reduce` | Opeenvolgende accumulatie van waarden |
| Geneste asynchrone verwerking | `mergeMap`, `switchMap`, `concatMap`, `exhaustMap` | Genereer en combineer Observables |
| Batchverwerking en groepering | `bufferTime`, `bufferCount`, `windowTime` | Collectieve verwerking en partitioneringsbeheer |
| Eigenschapextractie | `pluck` | Extraheer specifieke velden uit objecten |

## Gebruikersinvoervalidatie en -conversie

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

// Invoerveld
const emailInput = document.createElement('input');
const emailStatus = document.createElement('p');
document.body.appendChild(emailInput);
document.body.appendChild(emailStatus);

// E-mailvalidatiefunctie
function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

// Invoerverwerking
fromEvent(emailInput, 'input')
  .pipe(
    debounceTime(400),
    map((event) => (event.target as HTMLInputElement).value.trim()),
    distinctUntilChanged(),
    map((email) => {
      if (!email) {
        return {
          isValid: false,
          message: 'Voer een e-mailadres in',
          value: email,
        };
      }

      if (!isValidEmail(email)) {
        return {
          isValid: false,
          message: 'Voer een geldig e-mailadres in',
          value: email,
        };
      }

      return {
        isValid: true,
        message: 'E-mailadres is geldig',
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

## Object array conversie en aggregatie

```ts
import { from } from 'rxjs';
import { map, toArray } from 'rxjs';

// Verkoopdata
const sales = [
  { product: 'Laptop', price: 120000, quantity: 3 },
  { product: 'Tablet', price: 45000, quantity: 7 },
  { product: 'Smartphone', price: 85000, quantity: 4 },
  { product: 'Muis', price: 3500, quantity: 12 },
  { product: 'Toetsenbord', price: 6500, quantity: 8 },
];

// Dataconversie en aggregatie
from(sales)
  .pipe(
    // Bereken totaalbedrag voor elk product
    map((item) => ({
      product: item.product,
      price: item.price,
      quantity: item.quantity,
      total: item.price * item.quantity,
    })),
    // Voeg prijs inclusief BTW toe
    map((item) => ({
      ...item,
      totalWithTax: Math.round(item.total * 1.1),
    })),
    // Converteer terug naar array
    toArray(),
    // Bereken eindtotaal
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
    console.log('Productdetails:', result.items);
    console.log('Eindtotaal (excl. BTW):', result.grandTotal);
    console.log('Eindtotaal (incl. BTW):', result.grandTotalWithTax);
  });
// Uitvoer:
// Productdetails: (5) [{…}, {…}, {…}, {…}, {…}]
// Eindtotaal (excl. BTW): 1109000
// Eindtotaal (incl. BTW): 1219900
```

## JSON data normalisatie

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
      // Converteer naar object met ID als sleutel
      const normalizedUsers: Record<number, any> = {};
      const userIds: number[] = [];

      users.forEach((user) => {
        normalizedUsers[user.id] = {
          ...user,
          // Plat geneste objecten
          companyName: user.company.name,
          city: user.address.city,
          street: user.address.street,
          // Verwijder onnodige nesting
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
    title.textContent = 'Genormaliseerde gebruikersdata';
    resultBox.appendChild(title);

    result.ids.forEach((id) => {
      const user = result.entities[id];
      const div = document.createElement('div');
      div.innerHTML = `
      <strong>${user.name}</strong><br>
      Gebruikersnaam: @${user.username}<br>
      E-mail: ${user.email}<br>
      Bedrijf: ${user.companyName}<br>
      Adres: ${user.city}, ${user.street}<br><br>
    `;
      resultBox.appendChild(div);
    });

    // Snelle toegang tot gebruiker met specifiek ID
    console.log('Gebruiker ID 3:', result.entities[3]);
  });

```

## Combineren van meerdere transformaties

In echte applicaties is het gebruikelijk om een combinatie van meerdere transformatieoperators te gebruiken.

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

// Zoekinvoer
const searchInput = document.createElement('input');
const resultsContainer = document.createElement('p');
const loadingIndicator = document.createElement('p');

document.body.append(searchInput);
document.body.append(resultsContainer);
document.body.append(loadingIndicator);

// Zoekverwerking
fromEvent(searchInput, 'input')
  .pipe(
    // Krijg invoerwaarde
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Wacht 300ms
    debounceTime(300),
    // Negeer dezelfde waarde
    distinctUntilChanged(),
    // Toon laden
    tap(() => {
      loadingIndicator.style.display = 'block';
      resultsContainer.innerHTML = '';
    }),
    // API-verzoek (annuleer vorig verzoek)
    switchMap((term) => {
      // Lege invoer retourneert geen resultaten
      if (term === '') {
        return [];
      }

      // Timeout afhandeling (5 seconden)
      const timeout$ = timer(5000).pipe(
        tap(() => console.warn('API-respons time-out')),
        map(() => [{ error: 'Time-out' }])
      );

      // API-aanroep
      const response$ = ajax
        .getJSON(
          `https://jsonplaceholder.typicode.com/users?username_like=${term}`
        )
        .pipe(
          // Verwerk resultaten
          map((users) =>
            (users as User[]).map((user) => ({
              id: user.id,
              name: user.name,
              username: user.username,
              email: user.email,
              company: user.company.name,
            }))
          ),
          // Voltooi voor time-out
          takeUntil(timeout$)
        );

      return response$;
    }),
    // Verberg laden
    tap(() => {
      loadingIndicator.style.display = 'none';
    })
  )
  .subscribe((result) => {
    loadingIndicator.style.display = 'none';

    if (Array.isArray(result)) {
      if (result.length === 0) {
        resultsContainer.innerHTML =
          '<div class="no-results">Geen gebruikers gevonden</div>';
      } else {
        resultsContainer.innerHTML = result
          .map(
            (user) => `
          <div class="user-card">
            <h3>${user.name}</h3>
            <p>@${user.username}</p>
            <p>${user.email}</p>
            <p>Bedrijf: ${user.company}</p>
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

## 🧠 Samenvatting

- Voor eenvoudige conversies, gebruik `map`
- Voor het afhandelen van asynchrone operaties, gebruik `mergeMap`, `switchMap`, `concatMap`, `exhaustMap`
- Voor batchverwerking, gebruik `bufferTime`, `bufferCount`
- Voor eigenschapextractie, gebruik `pluck`
- **In echte apps is het combineren van deze operators de norm**

Als u de transformatieoperators beheerst, kunt u complexe asynchrone datastromen
intuïtief en declaratief afhandelen!
