---
description: Der Mechanismus von RxJS Multicasting wird detailliert erklärt. Grundlegende Patterns mit Subject, Verwendungsunterschiede zwischen share und shareReplay-Operatoren, Vermeidung doppelter API-Anfragen, Cache-Strategien, Teilen des Anwendungszustands und andere praktische Designpatterns werden mit TypeScript-Codebeispielen vorgestellt.
---

# Der Mechanismus von Multicasting

Multicasting ist eine Technik zur effizienten Verteilung eines Datenstroms von einem Observable an mehrere Abonnenten (Observer).
In RxJS kann dies durch Subjects oder Operatoren realisiert werden.

## Was ist Multicasting

Ein normales Observable (Cold Observable) erstellt bei jedem Abonnement einen neuen Datenstrom. Dies bedeutet, dass bei mehreren Abonnenten dieselbe Verarbeitung mehrmals ausgeführt wird.

Mit Multicasting kann die Datenquelle nur einmal ausgeführt und das Ergebnis an mehrere Abonnenten verteilt werden. Dies ist besonders wichtig in folgenden Fällen:

- HTTP/API-Anfragen sollen nicht doppelt aufgerufen werden
- Kostspielige Operationen (Berechnungen oder Nebeneffekte) sollen nur einmal ausgeführt werden
- Anwendungszustand soll über mehrere Komponenten geteilt werden

## Grundlegende Multicasting-Patterns

### Grundlegendes Multicast mit Subject

```ts
import { Observable, Subject } from 'rxjs';
import { tap } from 'rxjs';

// Datenquelle (Cold Observable)
function createDataSource(): Observable<number> {
  return new Observable<number>(observer => {
    console.log('Datenquelle: Verbindung');
    // Datengenerierungslogik (angenommen als kostspielige Operation)
    const id = setInterval(() => {
      const value = Math.round(Math.random() * 100);
      console.log(`Datenquelle: Wert generiert -> ${value}`);
      observer.next(value);
    }, 1000);

    // Cleanup-Funktion
    return () => {
      console.log('Datenquelle: Trennung');
      clearInterval(id);
    };
  });
}

// Multicast-Implementierung
function multicast() {
  // Original-Datenquelle
  const source$ = createDataSource().pipe(
    tap(value => console.log(`Quellverarbeitung: ${value}`))
  );

  // Subject für Multicast
  const subject = new Subject<number>();

  // Quelle mit Subject verbinden
  const subscription = source$.subscribe(subject);

  // Mehrere Abonnenten abonnieren das Subject
  console.log('Observer 1 Abonnement gestartet');
  const subscription1 = subject.subscribe(value => console.log(`Observer 1: ${value}`));

  // Nach 3 Sekunden weiteren Abonnenten hinzufügen
  setTimeout(() => {
    console.log('Observer 2 Abonnement gestartet');
    const subscription2 = subject.subscribe(value => console.log(`Observer 2: ${value}`));

    // Nach 5 Sekunden alle Abonnements beenden
    setTimeout(() => {
      console.log('Alle Abonnements beenden');
      subscription.unsubscribe();
      subscription1.unsubscribe();
      subscription2.unsubscribe();
    }, 5000);
  }, 3000);
}

// Ausführen
multicast();
```

#### Ausführungsergebnis
```
Datenquelle: Verbindung
Observer 1 Abonnement gestartet
Datenquelle: Wert generiert -> 71
Quellverarbeitung: 71
Observer 1: 71
Datenquelle: Wert generiert -> 79
Quellverarbeitung: 79
Observer 1: 79
Datenquelle: Wert generiert -> 63
Quellverarbeitung: 63
Observer 1: 63
Observer 2 Abonnement gestartet
Datenquelle: Wert generiert -> 49
Quellverarbeitung: 49
Observer 1: 49
Observer 2: 49
Datenquelle: Wert generiert -> 94
Quellverarbeitung: 94
Observer 1: 94
Observer 2: 94
Datenquelle: Wert generiert -> 89
Quellverarbeitung: 89
Observer 1: 89
Observer 2: 89
Datenquelle: Wert generiert -> 10
Quellverarbeitung: 10
Observer 1: 10
Observer 2: 10
Datenquelle: Wert generiert -> 68
Quellverarbeitung: 68
Observer 1: 68
Observer 2: 68
Alle Abonnements beenden
Datenquelle: Trennung
```

## Multicast-Operatoren

RxJS bietet dedizierte Operatoren zur Implementierung von Multicasting.

### `share()` Operator
[📘 RxJS Official: share()](https://rxjs.dev/api/index/function/share)

Der einfachste Operator zur Implementierung von Multicast.
Intern kombiniert er `multicast()` und `refCount()`.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Observable, das mit Intervall zählt
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Quelle: ${value}`)),
  share() // Multicast aktivieren
);

// Erster Abonnent
console.log('Observer 1 Abonnement gestartet');
const subscription1 = source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Nach 2,5 Sekunden zweiten Abonnenten hinzufügen
setTimeout(() => {
  console.log('Observer 2 Abonnement gestartet');
  const subscription2 = source$.subscribe(value => console.log(`Observer 2: ${value}`));

  // Nach 5 Sekunden Abonnent 1 beenden
  setTimeout(() => {
    console.log('Observer 1 Abonnement beenden');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

#### Ausführungsergebnis
```
Observer 1 Abonnement gestartet
Quelle: 0
Observer 1: 0
Observer 2 Abonnement gestartet
Quelle: 1
Observer 1: 1
Observer 2: 1
Quelle: 2
Observer 1: 2
Observer 2: 2
Quelle: 3
Observer 1: 3
Observer 2: 3
Observer 1 Abonnement beenden
Quelle: 4
Observer 2: 4
```

### Detaillierte Kontrolle von `share()`

Anstelle von `refCount()` kann in RxJS 7 und später das Verhalten klarer gesteuert werden, indem Optionen an `share()` übergeben werden.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Quelle: ${value}`)),
  share({
    resetOnError: true,
    resetOnComplete: true,
    resetOnRefCountZero: true,
  })
);

// Erster Abonnent
console.log('Observer 1 Abonnement gestartet');
const subscription1 = source$.subscribe((value) =>
  console.log(`Observer 1: ${value}`)
);

// Nach 2,5 Sekunden zweiten Abonnenten hinzufügen
setTimeout(() => {
  console.log('Observer 2 Abonnement gestartet');
  const subscription2 = source$.subscribe((value) =>
    console.log(`Observer 2: ${value}`)
  );

  setTimeout(() => {
    console.log('Observer 1 Abonnement beenden');
    subscription1.unsubscribe();
  }, 1500);
}, 2500);
```

#### Ausführungsergebnis
```
Observer 1 Abonnement gestartet
Quelle: 0
Observer 1: 0
Quelle: 1
Observer 1: 1
Observer 2 Abonnement gestartet
Quelle: 2
Observer 1: 2
Observer 2: 2
Quelle: 3
Observer 1: 3
Observer 2: 3
Observer 1 Abonnement beenden
Quelle: 4
Observer 2: 4
Quelle: 5
Observer 2: 5
```

Auf diese Weise kann das Verhalten bei Stream-Ende oder wenn Abonnenten auf Null gehen, klar gesteuert werden.

### `shareReplay()` Operator

[📘 RxJS Official: shareReplay()](https://rxjs.dev/api/index/function/shareReplay)

Ähnlich wie `share()`, aber speichert eine bestimmte Anzahl vergangener Werte und stellt sie auch späteren Abonnenten zur Verfügung.

```ts
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Verwendung von shareReplay (Puffergröße 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Quelle: ${value}`)),
  shareReplay(2) // Puffert die letzten 2 Werte
);

// Erster Abonnent
console.log('Observer 1 Abonnement gestartet');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Nach 3,5 Sekunden zweiten Abonnenten hinzufügen
setTimeout(() => {
  console.log('Observer 2 Abonnement gestartet - empfängt die letzten 2 Werte');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

#### Ausführungsergebnis
```
Observer 1 Abonnement gestartet
Quelle: 0
Observer 1: 0
Quelle: 1
Observer 1: 1
Observer 2 Abonnement gestartet - empfängt die letzten 2 Werte
Observer 2: 0
Observer 2: 1
Quelle: 2
Observer 1: 2
Observer 2: 2
Quelle: 3
Observer 1: 3
Observer 2: 3
Quelle: 4
Observer 1: 4
Observer 2: 4
```

## Timing und Lebenszyklus beim Multicasting

Es ist wichtig, den Lebenszyklus von Multicast-Streams zu verstehen. Insbesondere bei Verwendung des `share()`-Operators ist folgendes Verhalten zu beachten:

1. Erster Abonnent: `share()` startet die Verbindung zum Quell-Observable, wenn das erste Abonnement erfolgt.
2. Alle Abonnenten beenden: Bei der Einstellung `share({ resetOnRefCountZero: true })` wird die Verbindung zur Quelle getrennt, wenn die Anzahl der Abonnenten auf Null geht.
3. Abschluss oder Fehler: Standardmäßig setzt `share()` den internen Zustand zurück, wenn complete oder error auftritt (wenn resetOnComplete/resetOnError true ist).
4. Erneutes Abonnement: Nach einem Stream-Reset wird beim erneuten Abonnement ein neues Observable rekonstruiert.

Auf diese Weise wird das Timing von Start, Stopp und Neuaufbau des Streams durch die Optionen von `share()` basierend auf der Anzahl der Abonnements und dem Abschlusszustand gesteuert.

## Praktische Anwendungsfälle

### API-Anfragen teilen

Beispiel zur Vermeidung doppelter Anfragen an denselben API-Endpunkt.

```ts
import { Observable, of, throwError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, catchError, shareReplay, tap } from 'rxjs';

// API-Service-Simulation
class UserService {
  private cache = new Map<string, Observable<any>>();

  getUser(id: string): Observable<any> {
    // Wenn im Cache vorhanden, zurückgeben
    if (this.cache.has(id)) {
      console.log(`Benutzer-ID ${id} aus Cache abrufen`);
      return this.cache.get(id)!;
    }

    // Neue Anfrage erstellen
    console.log(`Benutzer-ID ${id} von API abrufen`);
    const request$ = ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${id}`).pipe(
      tap(response => console.log('API-Antwort:', response)),
      catchError(error => {
        console.error('API-Fehler:', error);
        // Aus Cache entfernen
        this.cache.delete(id);
        return throwError(() => new Error('Benutzerabruf fehlgeschlagen'));
      }),
      // Mit shareReplay teilen (Wert auch nach Abschluss cachen)
      shareReplay(1)
    );

    // Im Cache speichern
    this.cache.set(id, request$);
    return request$;
  }
}

// Verwendungsbeispiel
const userService = new UserService();

// Mehrere Komponenten fordern dieselben Benutzerdaten an
console.log('Komponente 1: Benutzerdaten anfordern');
userService.getUser('1').subscribe(user => {
  console.log('Komponente 1: Benutzerdaten empfangen', user);
});

// Etwas später fordert eine andere Komponente dieselben Daten an
setTimeout(() => {
  console.log('Komponente 2: dieselben Benutzerdaten anfordern');
  userService.getUser('1').subscribe(user => {
    console.log('Komponente 2: Benutzerdaten empfangen', user);
  });
}, 1000);

// Anderen Benutzer anfordern
setTimeout(() => {
  console.log('Komponente 3: andere Benutzerdaten anfordern');
  userService.getUser('2').subscribe(user => {
    console.log('Komponente 3: Benutzerdaten empfangen', user);
  });
}, 2000);
```

#### Ausführungsergebnis
```
Komponente 1: Benutzerdaten anfordern
Benutzer-ID 1 von API abrufen
API-Antwort: {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Komponente 1: Benutzerdaten empfangen {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Komponente 2: dieselben Benutzerdaten anfordern
Benutzer-ID 1 aus Cache abrufen
Komponente 2: Benutzerdaten empfangen {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Komponente 3: andere Benutzerdaten anfordern
Benutzer-ID 2 von API abrufen
API-Antwort: {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
Komponente 3: Benutzerdaten empfangen {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
```

## Multicasting-Designpatterns

### Singleton Observable

Pattern zum Teilen eines einzelnen Observables über die gesamte Anwendung hinweg.

```ts
import { Subject } from 'rxjs';

// Globale Zustandsverwaltung der Anwendung
class AppState {
  // Singleton-Instanz
  private static instance: AppState;

  // Globaler Benachrichtigungsstream
  private notificationsSubject = new Subject<string>();

  // Observable zur Veröffentlichung (read-only)
  readonly notifications$ = this.notificationsSubject.asObservable();

  // Singleton-Zugriff
  static getInstance(): AppState {
    if (!AppState.instance) {
      AppState.instance = new AppState();
    }
    return AppState.instance;
  }

  // Methode zum Senden von Benachrichtigungen
  notify(message: string): void {
    this.notificationsSubject.next(message);
  }
}

// Verwendungsbeispiel
const appState = AppState.getInstance();

// Benachrichtigungen überwachen (von mehreren Komponenten)
appState.notifications$.subscribe((msg) =>
  console.log('Komponente A:', msg)
);
appState.notifications$.subscribe((msg) =>
  console.log('Komponente B:', msg)
);

// Benachrichtigung senden
appState.notify('Systemaktualisierung verfügbar');
```

#### Ausführungsergebnis
```ts
Komponente A: Systemaktualisierung verfügbar
Komponente B: Systemaktualisierung verfügbar
```

## Zusammenfassung

Multicasting ist eine wichtige Technik zur Verbesserung der Effizienz und Performance von RxJS-Anwendungen. Die Hauptpunkte sind:

- Multicasting ermöglicht das Teilen einer Datenquelle mit mehreren Abonnenten
- Kann mit Operatoren wie `share()`, `shareReplay()`, `publish()` implementiert werden
- Kann doppelte API-Anfragen vermeiden und rechenintensive Verarbeitungen optimieren
- Hilfreich für State Management und Kommunikation zwischen Komponenten

Durch die Wahl einer geeigneten Multicast-Strategie kann die Reaktionsfähigkeit und Effizienz der Anwendung erhöht werden, während gleichzeitig die Codemenge reduziert und die Wartbarkeit verbessert wird.
