---
description: switchMap ist ein Transformationsoperator, der das vorherige Observable abbricht und zum neuesten wechselt. Ideal für Anwendungsfälle wie Live-Suche, Navigationsumschaltung und Auto-Save. Realisiert sichere asynchrone Verarbeitung mit TypeScript-Typinferenz. Ausführliche Erklärung zur Unterscheidung von mergeMap und concatMap.
---

# switchMap - Zum Neuesten wechseln

Der `switchMap`-Operator erzeugt für jeden Wert des Eingabestreams ein neues Observable und **bricht das vorherige Observable ab, um nur zum neuesten Observable zu wechseln**.
Ideal für Fälle wie Suchformulare, bei denen nur die aktuellste Eingabe gültig sein soll.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of } from 'rxjs';
import { delay, switchMap } from 'rxjs';

of('A', 'B', 'C').pipe(
  switchMap(value =>
    of(`${value} abgeschlossen`).pipe(delay(1000))
  )
).subscribe(console.log);

// Ausgabebeispiel:
// C abgeschlossen
```

- Für jeden Wert wird ein neues Observable erstellt.
- Sobald jedoch ein neuer Wert eintrifft, wird **das vorherige Observable sofort abgebrochen**.
- Letztendlich wird nur `C` ausgegeben.

[🌐 RxJS Offizielle Dokumentation - `switchMap`](https://rxjs.dev/api/operators/switchMap)

## 💡 Typische Anwendungsmuster

- Autovervollständigung in Eingabeformularen
- Live-Suchfunktion (nur die neueste Eingabe ist gültig)
- Laden von Ressourcen beim Wechsel der Navigation oder des Routings
- Wenn Benutzeraktionen auf die neueste umgeschaltet werden sollen

## 🧠 Praktisches Codebeispiel (mit UI)

Wenn Zeichen in das Suchfeld eingegeben werden, wird sofort eine API-Anfrage gesendet und **nur das Ergebnis der letzten Eingabe** angezeigt.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

// Eingabefeld erstellen
const searchInput = document.createElement('input');
searchInput.placeholder = 'Nach Benutzername suchen';
document.body.appendChild(searchInput);

// Ausgabebereich
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Eingabeereignisverarbeitung
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

- Bei jeder Änderung der Eingabe wird die vorherige Anfrage abgebrochen.
- Es werden nur Benutzer angezeigt, die zum neuesten Suchwort passen.
