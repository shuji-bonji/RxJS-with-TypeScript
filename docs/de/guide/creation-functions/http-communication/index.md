---
description: "HTTP-Kommunikations-Creation-Functions in RxJS: ajax und fromFetch. Vergleich, Unterschiede, Fehlerbehandlung und Einsatzrichtlinien für typsichere HTTP-Kommunikation."
---

# HTTP-Kommunikations-Creation-Functions

In RxJS werden Creation Functions bereitgestellt, um HTTP-Kommunikation als Observable zu behandeln. In diesem Abschnitt werden die beiden Funktionen `ajax()` und `fromFetch()` ausführlich erläutert.

## Was sind HTTP-Kommunikations-Creation-Functions?

HTTP-Kommunikations-Creation-Functions sind eine Gruppe von Funktionen, die es ermöglichen, die Kommunikation mit externen APIs oder Servern als Observable-Streams zu behandeln. Durch ihre Verwendung kann asynchrone HTTP-Kommunikation in RxJS-Operatorketten integriert und Fehlerbehandlung sowie Retry-Verarbeitung deklarativ beschrieben werden.

### Hauptmerkmale

- **Deklarative HTTP-Kommunikation**: Durch die Behandlung von HTTP-Kommunikation als Observable ist eine deklarative Verarbeitung mit Operatoren möglich
- **Einheitliche Fehlerbehandlung**: Fehlerverarbeitung kann mit Operatoren wie `catchError()` oder `retry()` vereinheitlicht werden
- **Stornierbar**: Anfragen können mit `unsubscribe()` storniert werden
- **Integration mit anderen Streams**: Kann mit Operatoren wie `switchMap()` mit anderen Observables kombiniert werden

## Liste der HTTP-Kommunikations-Creation-Functions

| Funktion | Beschreibung | Basistechnologie | Hauptverwendung |
|----------|--------------|------------------|-----------------|
| [ajax()](/de/guide/creation-functions/http-communication/ajax) | XMLHttpRequest-basierte HTTP-Kommunikation | XMLHttpRequest | Legacy-Browser-Unterstützung, Fortschrittsüberwachung |
| [fromFetch()](/de/guide/creation-functions/http-communication/fromFetch) | Fetch-API-basierte HTTP-Kommunikation | Fetch API | Moderne Browser, leichte HTTP-Kommunikation |

## Vergleich ajax() vs fromFetch()

### Grundlegende Unterschiede

```typescript
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { fromFetch } from 'rxjs/fetch';

// ajax() - Parst Response automatisch
const ajax$ = ajax.getJSON<Todo>('https://jsonplaceholder.typicode.com/todos/1');
ajax$.subscribe(data => console.log(data));

// fromFetch() - Manuelles Parsen der Response erforderlich
const fetch$ = fromFetch('https://jsonplaceholder.typicode.com/todos/1').pipe(
  switchMap(response => response.json())
);
fetch$.subscribe(data => console.log(data));

interface Todo {
  userId: number;
  id: number;
  title: string;
  completed: boolean;
}
```

### Funktionsvergleichstabelle

| Funktion | ajax() | fromFetch() |
|----------|--------|-------------|
| Basistechnologie | XMLHttpRequest | Fetch API |
| Automatisches JSON-Parsing | ✅ Mit `getJSON()` unterstützt | ❌ Manueller `.json()`-Aufruf erforderlich |
| Fortschrittsereignisse | ✅ Unterstützt | ❌ Nicht unterstützt |
| Timeout | ✅ Integrierte Unterstützung | ❌ Manuelle Implementierung erforderlich |
| Automatische HTTP-Fehlererkennung | ✅ Automatischer Fehler bei 4xx/5xx | ❌ Manueller Statuscheck erforderlich |
| Anfragestornierung | ✅ Mit unsubscribe() möglich | ✅ Mit unsubscribe() möglich |
| IE11-Unterstützung | ✅ Unterstützt | ❌ Polyfill erforderlich |
| Bundle-Größe | Etwas größer | Klein |

## Richtlinien zur Auswahl

### Wann ajax() zu wählen ist

1. **Legacy-Browser-Unterstützung erforderlich**
   - Wenn ältere Browser wie IE11 unterstützt werden müssen

2. **Fortschrittsüberwachung erforderlich**
   - Wenn der Fortschritt von Datei-Uploads/Downloads angezeigt werden soll

3. **Einfacher JSON-Abruf**
   - Wenn JSON einfach mit `getJSON()` abgerufen werden soll

4. **Automatische Fehlererkennung erforderlich**
   - Wenn automatische Fehlererkennung anhand von HTTP-Statuscodes verwendet werden soll

### Wann fromFetch() zu wählen ist

1. **Nur moderne Browser unterstützt**
   - Wenn nur Umgebungen unterstützt werden, in denen die Fetch API verfügbar ist

2. **Bundle-Größe klein halten**
   - Wenn leichte HTTP-Kommunikationsfunktionen ausreichen

3. **Fetch-API-Funktionen verwenden**
   - Wenn Request/Response-Objekte direkt manipuliert werden sollen
   - Wenn innerhalb eines Service Workers verwendet werden soll

4. **Feinere Kontrolle erforderlich**
   - Wenn die Response-Verarbeitung fein angepasst werden soll

## Praktische Verwendungsbeispiele

### API-Aufrufmuster

```typescript
import { of, catchError, retry, timeout } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface User {
  id: number;
  name: string;
  email: string;
}

// Praktisches Muster mit ajax()
const fetchUser$ = ajax.getJSON<User>('https://api.example.com/users/1').pipe(
  timeout(5000), // Timeout nach 5 Sekunden
  retry(2), // Bei Fehler 2 Mal wiederholen
  catchError(error => {
    console.error('Fehler beim Abrufen des Benutzers:', error);
    return of(null); // Bei Fehler null zurückgeben
  })
);

fetchUser$.subscribe({
  next: user => {
    if (user) {
      console.log('Benutzer:', user);
    } else {
      console.log('Abrufen des Benutzers fehlgeschlagen');
    }
  }
});
```

### Formularübermittlungsmuster

```typescript
import { fromEvent, switchMap, map } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Formular-Submit-Event als Observable
const form = document.querySelector('form') as HTMLFormElement;
const submit$ = fromEvent(form, 'submit').pipe(
  map(event => {
    event.preventDefault();
    const formData = new FormData(form);
    return Object.fromEntries(formData.entries());
  }),
  switchMap(data =>
    ajax.post('https://api.example.com/submit', data, {
      'Content-Type': 'application/json'
    })
  )
);

submit$.subscribe({
  next: response => console.log('Übermittlung erfolgreich:', response),
  error: error => console.error('Übermittlungsfehler:', error)
});
```

## Häufig gestellte Fragen

### F1: Sollte ich ajax() oder fromFetch() verwenden?

**A:** Für reine moderne Browser wird `fromFetch()` empfohlen. Gründe:
- Fetch API ist der neueste Web-Standard
- Bundle-Größe ist klein
- Höhere Zukunftskompatibilität

Wählen Sie jedoch `ajax()` in folgenden Fällen:
- IE11-Unterstützung erforderlich
- Fortschrittsüberwachung erforderlich
- Einfacher JSON-Abruf ausreichend

### F2: Wie werden HTTP-Fehler (4xx, 5xx) behandelt?

**A:**
- **ajax()**: Bei HTTP-Statuscodes ab 400 wird automatisch als Fehler behandelt und der `error`-Callback aufgerufen
- **fromFetch()**: Auch bei HTTP-Fehlern wird der `next`-Callback aufgerufen. `response.ok` muss manuell überprüft werden

### F3: Wie storniert man Anfragen?

**A:** Beide können mit `unsubscribe()` storniert werden.

```typescript
const subscription = ajax.getJSON('/api/data').subscribe(...);

// Nach 3 Sekunden stornieren
setTimeout(() => subscription.unsubscribe(), 3000);
```

## Nächste Schritte

Für detaillierte Verwendung jeder Funktion siehe folgende Seiten:

- [ajax() Details](/de/guide/creation-functions/http-communication/ajax) - XMLHttpRequest-basierte HTTP-Kommunikation
- [fromFetch() Details](/de/guide/creation-functions/http-communication/fromFetch) - Fetch-API-basierte HTTP-Kommunikation

## Referenzressourcen

- [RxJS Offizielle Dokumentation - ajax](https://rxjs.dev/api/ajax/ajax)
- [RxJS Offizielle Dokumentation - fromFetch](https://rxjs.dev/api/fetch/fromFetch)
- [MDN Web Docs - Fetch API](https://developer.mozilla.org/de/docs/Web/API/Fetch_API)
- [MDN Web Docs - XMLHttpRequest](https://developer.mozilla.org/de/docs/Web/API/XMLHttpRequest)
