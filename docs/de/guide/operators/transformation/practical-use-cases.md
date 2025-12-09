---
description: "Praktische Muster für RxJS-Transformationsoperatoren: Benutzereingaben, API-Calls, Datenverarbeitung und Aggregation mit TypeScript-Beispielen."
---

# Praktische Transformationsmuster

Transformationsoperatoren sind eine der am häufigsten verwendeten Operatorengruppen in RxJS.
Sie spielen eine unverzichtbare Rolle bei der flexiblen Verarbeitung und Transformation von Daten in der reaktiven Programmierung.

In diesem Abschnitt werden typische praktische Beispiele vorgestellt und Anwendungsmuster für Transformationsoperatoren organisiert.


## 💬 Typische Anwendungsmuster

| Muster | Repräsentative Operatoren | Beschreibung |
|:---|:---|:---|
| Einfache Werttransformation | `map` | Wendet eine Transformationsfunktion auf jeden Wert an |
| Akkumulation/Aggregation | `scan`, `reduce` | Akkumuliert Werte schrittweise |
| Verschachtelte asynchrone Verarbeitung | `mergeMap`, `switchMap`, `concatMap`, `exhaustMap` | Erzeugt und kombiniert Observables |
| Batch-Verarbeitung/Gruppierung | `bufferTime`, `bufferCount`, `windowTime` | Verarbeitet zusammengefasst/verwaltet aufgeteilt |
| Eigenschaftsextraktion | `pluck` | Extrahiert bestimmte Felder aus Objekten |


## Validierung und Transformation von Benutzereingaben

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

// Eingabefeld
const emailInput = document.createElement('input');
const emailStatus = document.createElement('p');
document.body.appendChild(emailInput);
document.body.appendChild(emailStatus);

// E-Mail-Validierungsfunktion
function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

// Eingabeverarbeitung
fromEvent(emailInput, 'input')
  .pipe(
    debounceTime(400),
    map((event) => (event.target as HTMLInputElement).value.trim()),
    distinctUntilChanged(),
    map((email) => {
      if (!email) {
        return {
          isValid: false,
          message: 'Bitte geben Sie eine E-Mail-Adresse ein',
          value: email,
        };
      }

      if (!isValidEmail(email)) {
        return {
          isValid: false,
          message: 'Bitte geben Sie eine gültige E-Mail-Adresse ein',
          value: email,
        };
      }

      return {
        isValid: true,
        message: 'E-Mail-Adresse ist gültig',
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

## Transformation und Aggregation von Objekt-Arrays

```ts
import { from } from 'rxjs';
import { map, toArray } from 'rxjs';

// Verkaufsdaten
const sales = [
  { product: 'Notebook', price: 120000, quantity: 3 },
  { product: 'Tablet', price: 45000, quantity: 7 },
  { product: 'Smartphone', price: 85000, quantity: 4 },
  { product: 'Maus', price: 3500, quantity: 12 },
  { product: 'Tastatur', price: 6500, quantity: 8 },
];

// Datentransformation und Aggregation
from(sales)
  .pipe(
    // Berechne den Gesamtbetrag für jedes Produkt
    map((item) => ({
      product: item.product,
      price: item.price,
      quantity: item.quantity,
      total: item.price * item.quantity,
    })),
    // Füge Preis inkl. Steuer hinzu
    map((item) => ({
      ...item,
      totalWithTax: Math.round(item.total * 1.1),
    })),
    // Zurück in Array konvertieren
    toArray(),
    // Berechne Gesamtsumme
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
    console.log('Produktdetails:', result.items);
    console.log('Gesamtbetrag (ohne Steuer):', result.grandTotal);
    console.log('Gesamtbetrag (inkl. Steuer):', result.grandTotalWithTax);
  });
// Ausgabe:
// Produktdetails: (5) [{…}, {…}, {…}, {…}, {…}]
// Gesamtbetrag (ohne Steuer): 1109000
// Gesamtbetrag (inkl. Steuer): 1219900
```

## Normalisierung von JSON-Daten

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
      // In Objekt mit ID als Schlüssel konvertieren
      const normalizedUsers: Record<number, any> = {};
      const userIds: number[] = [];

      users.forEach((user) => {
        normalizedUsers[user.id] = {
          ...user,
          // Verschachtelte Objekte abflachen
          companyName: user.company.name,
          city: user.address.city,
          street: user.address.street,
          // Unnötige Verschachtelung entfernen
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
    title.textContent = 'Normalisierte Benutzerdaten';
    resultBox.appendChild(title);

    result.ids.forEach((id) => {
      const user = result.entities[id];
      const div = document.createElement('div');
      div.innerHTML = `
      <strong>${user.name}</strong><br>
      Benutzername: @${user.username}<br>
      E-Mail: ${user.email}<br>
      Firma: ${user.companyName}<br>
      Adresse: ${user.city}, ${user.street}<br><br>
    `;
      resultBox.appendChild(div);
    });

    // Schneller Zugriff auf Benutzer mit bestimmter ID möglich
    console.log('Benutzer-ID 3:', result.entities[3]);
  });

```

## Kombination mehrerer Transformationen

In realen Anwendungen werden häufig mehrere Transformationsoperatoren kombiniert verwendet.

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

// Sucheingabe
const searchInput = document.createElement('input');
const resultsContainer = document.createElement('p');
const loadingIndicator = document.createElement('p');

document.body.append(searchInput);
document.body.append(resultsContainer);
document.body.append(loadingIndicator);

// Suchverarbeitung
fromEvent(searchInput, 'input')
  .pipe(
    // Eingabewert abrufen
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // 300ms warten
    debounceTime(300),
    // Gleichen Wert ignorieren
    distinctUntilChanged(),
    // Ladeanzeige anzeigen
    tap(() => {
      loadingIndicator.style.display = 'block';
      resultsContainer.innerHTML = '';
    }),
    // API-Anfrage (vorherige Anfrage wird abgebrochen)
    switchMap((term) => {
      // Leere Eingabe ergibt keine Ergebnisse
      if (term === '') {
        return [];
      }

      // Timeout-Verarbeitung (5 Sekunden)
      const timeout$ = timer(5000).pipe(
        tap(() => console.warn('API-Antwort hat Timeout überschritten')),
        map(() => [{ error: 'Timeout' }])
      );

      // API-Aufruf
      const response$ = ajax
        .getJSON(
          `https://jsonplaceholder.typicode.com/users?username_like=${term}`
        )
        .pipe(
          // Ergebnisse verarbeiten
          map((users) =>
            (users as User[]).map((user) => ({
              id: user.id,
              name: user.name,
              username: user.username,
              email: user.email,
              company: user.company.name,
            }))
          ),
          // Bis zum Timeout abschließen
          takeUntil(timeout$)
        );

      return response$;
    }),
    // Laden beenden
    tap(() => {
      loadingIndicator.style.display = 'none';
    })
  )
  .subscribe((result) => {
    loadingIndicator.style.display = 'none';

    if (Array.isArray(result)) {
      if (result.length === 0) {
        resultsContainer.innerHTML =
          '<div class="no-results">Keine Benutzer gefunden</div>';
      } else {
        resultsContainer.innerHTML = result
          .map(
            (user) => `
          <div class="user-card">
            <h3>${user.name}</h3>
            <p>@${user.username}</p>
            <p>${user.email}</p>
            <p>Firma: ${user.company}</p>
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

## 🧠 Zusammenfassung

- Für einfache Transformationen verwenden Sie `map`
- Für asynchrone Verarbeitung verwenden Sie `mergeMap`・`switchMap`・`concatMap`・`exhaustMap`
- Für Batch-Verarbeitung verwenden Sie `bufferTime`・`bufferCount`
- Für Eigenschaftsextraktion verwenden Sie `pluck`
- In realen Anwendungen ist **die Kombination dieser Operatoren die Norm**

Wenn Sie Transformationsoperatoren beherrschen, können Sie auch komplexe asynchrone Datenflüsse
intuitiv und deklarativ handhaben!
