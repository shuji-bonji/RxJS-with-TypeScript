---
description: "Praktische Caching-Strategien mit RxJS. Daten-Caching mit shareReplay, Cache mit TTL, Cache-Invalidierung, LocalStorage-Integration und Offline-First-Design für effizientes Datenmanagement in TypeScript."
---

# Caching-Strategiemuster

Caching ist eine der wichtigsten Techniken zur Leistungsoptimierung. Mit RxJS können Sie deklarative und flexible Caching-Strategien implementieren.

Dieser Artikel erklärt konkrete Caching-Strategiemuster, die in der Praxis benötigt werden, vom grundlegenden Caching mit shareReplay über Cache mit TTL, Cache-Invalidierung bis hin zur Integration mit LocalStorage.

## Was Sie in diesem Artikel lernen werden

- Grundlegendes Caching mit shareReplay
- Implementierung von Cache mit TTL (Time To Live)
- Manuelles Refresh und Cache-Invalidierung
- Integration mit LocalStorage
- Offline-Unterstützung und Cache-Fallback
- Cache-Überwachung und Debugging

> [!TIP] Voraussetzungen
> Dieser Artikel setzt Kenntnisse aus [Kapitel 2: Cold/Hot Observable](../observables/cold-and-hot-observables.md) und [Kapitel 4: Operatoren](../operators/index.md) voraus. Das Verständnis von `shareReplay` und `share` ist besonders wichtig.

## Grundlegendes Caching (shareReplay)

### Problem: Vermeidung mehrfacher API-Aufrufe

Wenn mehrere Komponenten dieselben API-Daten benötigen, möchten Sie doppelte Anfragen verhindern.

### Lösung: Caching mit shareReplay

```typescript
import { Observable, of, shareReplay, catchError, tap } from 'rxjs';
interface User {
  id: number;
  name: string;
  email: string;
}

class UserService {
  private users$: Observable<User[]> | null = null;

  getUsers(): Observable<User[]> {
    // Wenn Cache vorhanden, diesen zurückgeben
    if (this.users$) {
      console.log('Rückgabe aus dem Cache');
      return this.users$;
    }

    // Neue Anfrage erstellen und cachen
    console.log('Neue Anfrage ausführen');
    this.users$ = this.fetchUsersFromAPI().pipe(
      tap(() => console.log('API-Aufruf abgeschlossen')),
      shareReplay(1), // Letzten Wert cachen
      catchError(err => {
        // Bei Fehler Cache löschen
        this.users$ = null;
        throw err;
      })
    );

    return this.users$;
  }

  clearCache(): void {
    this.users$ = null;
    console.log('Cache gelöscht');
  }

  private fetchUsersFromAPI(): Observable<User[]> {
    return of([
      { id: 1, name: 'Yamada Taro', email: 'yamada@example.com' },
      { id: 2, name: 'Sato Hanako', email: 'sato@example.com' }
    ]);
  }
}

// Verwendungsbeispiel
const userService = new UserService();

// Erster Aufruf (API-Ausführung)
userService.getUsers().subscribe(users => {
  console.log('Komponente 1:', users);
});

// Zweiter Aufruf (aus Cache)
userService.getUsers().subscribe(users => {
  console.log('Komponente 2:', users);
});

// Ausgabe:
// Neue Anfrage ausführen
// API-Aufruf abgeschlossen
// Komponente 1: [...]
// Rückgabe aus dem Cache
// Komponente 2: [...]
```

> [!IMPORTANT] Hinweise zu shareReplay
> - **Memory Leak**: Cache wird auch bei 0 Abonnenten beibehalten
> - **Referenztyp-Freigabe**: Objekte werden als Referenz geteilt, Änderungen betreffen alle Abonnenten
> - **Fehlerbehandlung**: Bei Fehlern wird empfohlen, den Cache zu löschen

### shareReplay Konfigurationsoptionen

```typescript
import { shareReplay } from 'rxjs';
// Grundlegende Verwendung
source$.pipe(
  shareReplay(1) // Letzten Wert cachen
);

// Detaillierte Konfiguration
source$.pipe(
  shareReplay({
    bufferSize: 1,        // Anzahl der zu cachenden Werte
    refCount: true,       // Cache verwerfen, wenn keine Abonnenten mehr vorhanden
    windowTime: 5000      // Cache nach 5 Sekunden verwerfen (optional)
  })
);
```

> [!TIP] Verwendung von refCount
> - `refCount: true` - Cache verwerfen, wenn keine Abonnenten (Speichereffizienz ◎)
> - `refCount: false` (Standard) - Dauerhafter Cache (Performance ◎)
>
> Wählen Sie je nach Verwendungszweck.

## Cache mit TTL (Time To Live)

### Problem: Automatische Invalidierung von altem Cache

Sie möchten Cache nach einer bestimmten Zeit automatisch verwerfen und neue Daten abrufen.

### Lösung: Kombination von Timestamp und filter

```typescript
import { Observable, of, shareReplay, map, switchMap } from 'rxjs';
interface CachedData<T> {
  data: T;
  timestamp: number;
}

class TTLCacheService<T> {
  private cache$: Observable<CachedData<T>> | null = null;
  private ttl: number; // Time To Live (Millisekunden)

  constructor(ttl: number = 60000) {
    this.ttl = ttl; // Standard: 60 Sekunden
  }

  getData(fetchFn: () => Observable<T>): Observable<T> {
    if (this.cache$) {
      // Cache auf Gültigkeit prüfen
      return this.cache$.pipe(
        switchMap(cached => {
          const age = Date.now() - cached.timestamp;
          if (age < this.ttl) {
            console.log(`Rückgabe aus Cache (${(this.ttl - age) / 1000} Sekunden bis Ablauf)`);
            return of(cached.data);
          } else {
            console.log('Cache abgelaufen - Neue Daten abrufen');
            this.cache$ = null;
            return this.getData(fetchFn);
          }
        })
      );
    }

    // Neue Daten abrufen und cachen
    console.log('Neue Anfrage ausführen');
    this.cache$ = fetchFn().pipe(
      map(data => ({
        data,
        timestamp: Date.now()
      })),
      shareReplay(1)
    );

    return this.cache$.pipe(map(cached => cached.data));
  }

  clearCache(): void {
    this.cache$ = null;
    console.log('Cache gelöscht');
  }

  getCacheAge(): number | null {
    // Cache-Alter abrufen (für Debugging)
    if (!this.cache$) return null;

    let timestamp = 0;
    this.cache$.subscribe(cached => {
      timestamp = cached.timestamp;
    });

    return Date.now() - timestamp;
  }
}

// Verwendungsbeispiel
interface Product {
  id: number;
  name: string;
  price: number;
}

const productCache = new TTLCacheService<Product[]>(30000); // 30 Sekunden TTL

function fetchProducts(): Observable<Product[]> {
  console.log('API-Aufruf');
  return of([
    { id: 1, name: 'Produkt A', price: 1000 },
    { id: 2, name: 'Produkt B', price: 2000 }
  ]);
}

// Erster Abruf (neue Daten)
productCache.getData(() => fetchProducts()).subscribe(products => {
  console.log('Abruf:', products);
});

// Nach 10 Sekunden (aus Cache)
setTimeout(() => {
  productCache.getData(() => fetchProducts()).subscribe(products => {
    console.log('Nach 10 Sekunden:', products);
    console.log('Cache-Alter:', productCache.getCacheAge(), 'ms');
  });
}, 10000);

// Nach 35 Sekunden (abgelaufen, neue Daten)
setTimeout(() => {
  productCache.getData(() => fetchProducts()).subscribe(products => {
    console.log('Nach 35 Sekunden (abgelaufen):', products);
  });
}, 35000);
```

**Funktionsweise von TTL-Cache:**

```mermaid
sequenceDiagram
    participant Component
    participant Cache
    participant API

    Component->>Cache: getData()
    Note over Cache: Kein Cache
    Cache->>API: fetch()
    API-->>Cache: data
    Cache-->>Component: data (Timestamp aufzeichnen)

    Note over Component,Cache: 10 Sekunden später
    Component->>Cache: getData()
    Note over Cache: Innerhalb TTL (weniger als 30 Sekunden)
    Cache-->>Component: cached data

    Note over Component,Cache: 35 Sekunden später
    Component->>Cache: getData()
    Note over Cache: TTL abgelaufen
    Cache->>API: fetch()
    API-->>Cache: new data
    Cache-->>Component: new data
```

## Manuelles Refresh und Cache-Invalidierung

### Problem: Benutzer möchte Daten beliebig aktualisieren

Beim Klicken auf eine „Aktualisieren"-Schaltfläche soll der Cache verworfen und die neuesten Daten abgerufen werden.

### Lösung: Steuerung mit Subject und switch

```typescript
import { Observable, Subject, merge, of, switchMap, shareReplay, tap } from 'rxjs';
class RefreshableCacheService<T> {
  private refreshTrigger$ = new Subject<void>();
  private cache$: Observable<T>;

  constructor(fetchFn: () => Observable<T>) {
    this.cache$ = merge(
      this.refreshTrigger$.pipe(
        tap(() => console.log('Manuelle Aktualisierung'))
      ),
      // Für erste Ausführung
      of(undefined).pipe(tap(() => console.log('Erstes Laden')))
    ).pipe(
      switchMap(() => fetchFn()),
      tap(data => console.log('Daten abgerufen:', data)),
      shareReplay(1)
    );
  }

  getData(): Observable<T> {
    return this.cache$;
  }

  refresh(): void {
    this.refreshTrigger$.next();
  }
}

const refreshButton = document.createElement('button');
refreshButton.id = 'refresh-button';
refreshButton.textContent = 'Nachrichten aktualisieren';
refreshButton.style.padding = '10px 20px';
refreshButton.style.margin = '10px';
refreshButton.style.fontSize = '16px';
refreshButton.style.fontWeight = 'bold';
refreshButton.style.color = '#fff';
refreshButton.style.backgroundColor = '#2196F3';
refreshButton.style.border = 'none';
refreshButton.style.borderRadius = '4px';
refreshButton.style.cursor = 'pointer';
document.body.appendChild(refreshButton);

const newsContainer = document.createElement('div');
newsContainer.id = 'news-container';
newsContainer.style.padding = '15px';
newsContainer.style.margin = '10px';
newsContainer.style.border = '2px solid #ccc';
newsContainer.style.borderRadius = '8px';
newsContainer.style.minHeight = '200px';
newsContainer.style.backgroundColor = '#f9f9f9';
document.body.appendChild(newsContainer);

const newsCache = new RefreshableCacheService<string[]>(() =>
  of(['Nachricht 1', 'Nachricht 2', 'Nachricht 3'])
);

// Datenabonnement
newsCache.getData().subscribe(news => {
  console.log('Nachrichtenliste:', news);
  displayNews(news, newsContainer);
});

// Klick auf Aktualisieren-Schaltfläche
refreshButton.addEventListener('click', () => {
  console.log('Benutzer hat Aktualisieren geklickt');
  refreshButton.textContent = 'Wird aktualisiert...';
  refreshButton.disabled = true;
  refreshButton.style.backgroundColor = '#999';
  newsCache.refresh();
  setTimeout(() => {
    refreshButton.textContent = 'Nachrichten aktualisieren';
    refreshButton.disabled = false;
    refreshButton.style.backgroundColor = '#2196F3';
  }, 1000);
});

function displayNews(news: string[], container: HTMLElement): void {
  container.innerHTML = news
    .map(item => `<div style="padding: 10px; margin: 5px 0; border-bottom: 1px solid #ddd; font-size: 14px;">${item}</div>`)
    .join('');

  if (news.length === 0) {
    container.innerHTML = '<div style="padding: 20px; text-align: center; color: #999;">Keine Nachrichten verfügbar</div>';
  }
}
```

### Bedingte Cache-Invalidierung

```typescript
import { BehaviorSubject, Observable, switchMap, shareReplay, distinctUntilChanged, of } from 'rxjs';
interface CacheOptions {
  forceRefresh: boolean;
  userId?: number;
}

class ConditionalCacheService {
  private options$ = new BehaviorSubject<CacheOptions>({
    forceRefresh: false
  });

  data$ = this.options$.pipe(
    distinctUntilChanged((prev, curr) => {
      // Neue Daten abrufen, wenn forceRefresh oder userId sich geändert hat
      return !curr.forceRefresh && prev.userId === curr.userId;
    }),
    switchMap(options => {
      console.log('Daten abrufen:', options);
      return this.fetchData(options.userId);
    }),
    shareReplay(1)
  );

  getData(userId?: number): Observable<any> {
    this.options$.next({
      forceRefresh: false,
      userId
    });
    return this.data$;
  }

  refresh(userId?: number): void {
    this.options$.next({
      forceRefresh: true,
      userId
    });
  }

  private fetchData(userId?: number): Observable<any> {
    console.log('API-Aufruf - userId:', userId);
    return of({ userId, data: 'sample data' });
  }
}

// Verwendungsbeispiel
const conditionalCache = new ConditionalCacheService();

// Daten für Benutzer 1 abrufen
conditionalCache.getData(1).subscribe(data => {
  console.log('Daten von Benutzer 1:', data);
});

// Gleicher Benutzer, daher aus Cache
conditionalCache.getData(1).subscribe(data => {
  console.log('Daten von Benutzer 1 (Cache):', data);
});

// Anderer Benutzer, daher neue Daten abrufen
conditionalCache.getData(2).subscribe(data => {
  console.log('Daten von Benutzer 2:', data);
});

// Manuelle Aktualisierung
conditionalCache.refresh(1);
```

## Integration mit LocalStorage

### Problem: Cache nach Seitenneuladung beibehalten

Sie möchten mit dem LocalStorage des Browsers einen dauerhaften Cache implementieren.

### Lösung: Kombination mit LocalStorage

```typescript
import { Observable, of, defer, tap, catchError } from 'rxjs';
interface StorageCacheOptions {
  key: string;
  ttl?: number; // Millisekunden
}

interface CachedItem<T> {
  data: T;
  timestamp: number;
}

class LocalStorageCacheService {
  getOrFetch<T>(
    options: StorageCacheOptions,
    fetchFn: () => Observable<T>
  ): Observable<T> {
    return defer(() => {
      // Versuch, aus LocalStorage abzurufen
      const cached = this.getFromStorage<T>(options.key, options.ttl);

      if (cached) {
        console.log('Aus LocalStorage abgerufen:', options.key);
        return of(cached);
      }

      // Kein Cache, neue Daten abrufen
      console.log('Neue Daten abrufen:', options.key);
      return fetchFn().pipe(
        tap(data => {
          this.saveToStorage(options.key, data);
        }),
        catchError(err => {
          console.error('Abruffehler:', err);
          throw err;
        })
      );
    });
  }

  private getFromStorage<T>(key: string, ttl?: number): T | null {
    try {
      const item = localStorage.getItem(key);
      if (!item) return null;

      const cached: CachedItem<T> = JSON.parse(item);

      // TTL-Prüfung
      if (ttl) {
        const age = Date.now() - cached.timestamp;
        if (age > ttl) {
          console.log('Cache abgelaufen:', key);
          localStorage.removeItem(key);
          return null;
        }
      }

      return cached.data;
    } catch (error) {
      console.error('LocalStorage-Lesefehler:', error);
      return null;
    }
  }

  private saveToStorage<T>(key: string, data: T): void {
    try {
      const item: CachedItem<T> = {
        data,
        timestamp: Date.now()
      };
      localStorage.setItem(key, JSON.stringify(item));
      console.log('In LocalStorage gespeichert:', key);
    } catch (error) {
      console.error('LocalStorage-Speicherfehler:', error);
    }
  }

  clearCache(key?: string): void {
    if (key) {
      localStorage.removeItem(key);
      console.log('Cache gelöscht:', key);
    } else {
      localStorage.clear();
      console.log('Gesamter Cache gelöscht');
    }
  }

  getCacheSize(): number {
    let size = 0;
    for (const key in localStorage) {
      if (localStorage.hasOwnProperty(key)) {
        size += localStorage[key].length;
      }
    }
    return size;
  }
}

// Verwendungsbeispiel
interface Settings {
  theme: string;
  language: string;
  notifications: boolean;
}

const storageCache = new LocalStorageCacheService();

function fetchSettings(): Observable<Settings> {
  console.log('Einstellungen von API abrufen');
  return of({
    theme: 'dark',
    language: 'de',
    notifications: true
  });
}

// Einstellungen abrufen (LocalStorage oder API)
storageCache.getOrFetch(
  { key: 'user-settings', ttl: 3600000 }, // 1 Stunde TTL
  fetchSettings
).subscribe(settings => {
  console.log('Einstellungen:', settings);
  applySettings(settings);
});

// Nach Seitenneuladung werden dieselben Daten abgerufen (innerhalb TTL)
// storageCache.getOrFetch(...) // Aus LocalStorage

function applySettings(settings: Settings): void {
  document.body.className = `theme-${settings.theme}`;
  console.log('Einstellungen angewendet:', settings);
}
```

### Storage-Größenverwaltung

```typescript
class ManagedStorageCacheService extends LocalStorageCacheService {
  private maxSize = 5 * 1024 * 1024; // 5MB

  saveWithLimit<T>(key: string, data: T): boolean {
    const item: CachedItem<T> = {
      data,
      timestamp: Date.now()
    };

    const itemString = JSON.stringify(item);
    const itemSize = new Blob([itemString]).size;

    // Wenn aktuelle Größe + neue Element-Größe Obergrenze überschreitet
    if (this.getCacheSize() + itemSize > this.maxSize) {
      console.log('Storage-Kapazitätsgrenze erreicht - Älteste Elemente löschen');
      this.removeOldestItem();
    }

    try {
      localStorage.setItem(key, itemString);
      return true;
    } catch (error) {
      console.error('Speichern fehlgeschlagen:', error);
      return false;
    }
  }

  private removeOldestItem(): void {
    let oldestKey: string | null = null;
    let oldestTimestamp = Date.now();

    for (const key in localStorage) {
      if (localStorage.hasOwnProperty(key)) {
        try {
          const item = JSON.parse(localStorage[key]);
          if (item.timestamp < oldestTimestamp) {
            oldestTimestamp = item.timestamp;
            oldestKey = key;
          }
        } catch (error) {
          // Parse-Fehler ignorieren
        }
      }
    }

    if (oldestKey) {
      localStorage.removeItem(oldestKey);
      console.log('Ältestes Element gelöscht:', oldestKey);
    }
  }
}
```

## Offline-Unterstützung

### Problem: Anzeige von Cache-Daten im Offline-Modus

Auch ohne Netzwerkverbindung möchten Sie gecachte Daten anzeigen, um die Benutzererfahrung zu verbessern.

### Lösung: Cache-First-Strategie

```typescript
import { Observable, of, throwError, fromEvent, merge, map, startWith, distinctUntilChanged, switchMap, catchError, tap } from 'rxjs';
class OfflineFirstCacheService {
  private onlineStatus$ = merge(
    fromEvent(window, 'online').pipe(map(() => true)),
    fromEvent(window, 'offline').pipe(map(() => false))
  ).pipe(
    startWith(navigator.onLine),
    distinctUntilChanged(),
    tap(online => console.log('Online-Status:', online))
  );

  getData<T>(
    cacheKey: string,
    fetchFn: () => Observable<T>
  ): Observable<T> {
    return this.onlineStatus$.pipe(
      switchMap(online => {
        if (online) {
          // Online: Von API abrufen und cachen
          console.log('Online - Von API abrufen');
          return fetchFn().pipe(
            tap(data => {
              this.saveToCache(cacheKey, data);
            }),
            catchError(err => {
              console.error('API-Abruffehler - Fallback auf Cache');
              return this.getFromCache<T>(cacheKey);
            })
          );
        } else {
          // Offline: Aus Cache abrufen
          console.log('Offline - Aus Cache abrufen');
          return this.getFromCache<T>(cacheKey);
        }
      })
    );
  }

  private saveToCache<T>(key: string, data: T): void {
    try {
      localStorage.setItem(key, JSON.stringify(data));
      console.log('Cache gespeichert:', key);
    } catch (error) {
      console.error('Cache-Speicherfehler:', error);
    }
  }

  private getFromCache<T>(key: string): Observable<T> {
    try {
      const cached = localStorage.getItem(key);
      if (cached) {
        const data = JSON.parse(cached);
        console.log('Aus Cache abgerufen:', key);
        return of(data);
      }
    } catch (error) {
      console.error('Cache-Lesefehler:', error);
    }

    return throwError(() => new Error('Cache nicht gefunden'));
  }
}

// Traditional approach (commented for reference)
// const container = document.querySelector('#articles');
// const message = document.querySelector('#offline-message');

// Self-contained: creates articles display dynamically
const articlesContainer = document.createElement('div');
articlesContainer.id = 'articles';
articlesContainer.style.padding = '15px';
articlesContainer.style.margin = '10px';
articlesContainer.style.border = '2px solid #ccc';
articlesContainer.style.borderRadius = '8px';
articlesContainer.style.backgroundColor = '#f9f9f9';
document.body.appendChild(articlesContainer);

const offlineMessage = document.createElement('div');
offlineMessage.id = 'offline-message';
offlineMessage.style.padding = '15px';
offlineMessage.style.margin = '10px';
offlineMessage.style.backgroundColor = '#f8d7da';
offlineMessage.style.color = '#721c24';
offlineMessage.style.border = '1px solid #f5c6cb';
offlineMessage.style.borderRadius = '4px';
offlineMessage.style.display = 'none';
offlineMessage.style.textAlign = 'center';
offlineMessage.style.fontWeight = 'bold';
document.body.appendChild(offlineMessage);

// Verwendungsbeispiel
const offlineCache = new OfflineFirstCacheService();

function fetchArticles(): Observable<any[]> {
  return of([
    { id: 1, title: 'Artikel 1', content: 'Inhalt 1' },
    { id: 2, title: 'Artikel 2', content: 'Inhalt 2' }
  ]);
}

offlineCache.getData('articles', fetchArticles).subscribe({
  next: articles => {
    console.log('Artikel:', articles);
    displayArticles(articles, articlesContainer);
    offlineMessage.style.display = 'none';
  },
  error: err => {
    console.error('Datenabruf fehlgeschlagen:', err);
    showOfflineMessage(offlineMessage);
  }
});

function displayArticles(articles: any[], container: HTMLElement): void {
  container.innerHTML = articles
    .map(a => `
      <article style="padding: 15px; margin: 10px 0; border-bottom: 2px solid #ddd;">
        <h2 style="margin: 0 0 10px 0; font-size: 18px; color: #333;">${a.title}</h2>
        <p style="margin: 0; font-size: 14px; color: #666;">${a.content}</p>
      </article>
    `)
    .join('');

  if (articles.length === 0) {
    container.innerHTML = '<div style="padding: 20px; text-align: center; color: #999;">Keine Artikel verfügbar</div>';
  }
}

function showOfflineMessage(message: HTMLElement): void {
  message.textContent = 'Sie sind offline. Keine Cache-Daten verfügbar.';
  message.style.display = 'block';
}
```

**Offline-Unterstützungsstrategie:**

```mermaid
flowchart TD
    A[Datenanfrage] --> B{Online?}
    B -->|Ja| C[API-Aufruf]
    C --> D{Erfolgreich?}
    D -->|Ja| E[Daten zurückgeben]
    D -->|Nein| F[Aus Cache abrufen]
    E --> G[In Cache speichern]
    F --> H[Cache-Daten zurückgeben]
    B -->|Nein| I[Aus Cache abrufen]
    I --> J{Cache vorhanden?}
    J -->|Ja| K[Cache-Daten zurückgeben]
    J -->|Nein| L[Fehler anzeigen]
```

## Cache-Überwachung und Debugging

### Visualisierung des Cache-Status

```typescript
import { BehaviorSubject, Observable, map } from 'rxjs';
interface CacheEntry {
  key: string;
  size: number;
  timestamp: number;
  hits: number;
}

interface CacheStats {
  entries: CacheEntry[];
  totalSize: number;
  hitRate: number;
}

class ObservableCacheService {
  private cacheEntries$ = new BehaviorSubject<Map<string, CacheEntry>>(new Map());
  private totalRequests = 0;
  private cacheHits = 0;

  stats$: Observable<CacheStats> = this.cacheEntries$.pipe(
    map(entries => {
      const entriesArray = Array.from(entries.values());
      const totalSize = entriesArray.reduce((sum, entry) => sum + entry.size, 0);
      const hitRate = this.totalRequests > 0
        ? (this.cacheHits / this.totalRequests) * 100
        : 0;

      return {
        entries: entriesArray,
        totalSize,
        hitRate
      };
    })
  );

  getData<T>(key: string, fetchFn: () => Observable<T>): Observable<T> {
    this.totalRequests++;

    const entries = this.cacheEntries$.value;
    const entry = entries.get(key);

    if (entry) {
      // Cache-Treffer
      this.cacheHits++;
      entry.hits++;
      this.cacheEntries$.next(new Map(entries));
      console.log(`Cache-Treffer: ${key} (${entry.hits}. Mal)`);
      // Tatsächliche Datenabruflogik
    } else {
      // Cache-Fehlschlag
      console.log(`Cache-Fehlschlag: ${key}`);
      // Neue Daten abrufen und Cache registrieren
      const newEntry: CacheEntry = {
        key,
        size: 0, // Tatsächliche Datengröße berechnen
        timestamp: Date.now(),
        hits: 1
      };
      entries.set(key, newEntry);
      this.cacheEntries$.next(new Map(entries));
    }

    return fetchFn();
  }

  clearStats(): void {
    this.totalRequests = 0;
    this.cacheHits = 0;
    this.cacheEntries$.next(new Map());
  }
}

// Traditional approach (commented for reference)
// const statsElement = document.querySelector('#cache-stats');

// Self-contained: creates cache stats element dynamically
const cacheStatsElement = document.createElement('div');
cacheStatsElement.id = 'cache-stats';
cacheStatsElement.style.padding = '20px';
cacheStatsElement.style.margin = '10px';
cacheStatsElement.style.border = '2px solid #ccc';
cacheStatsElement.style.borderRadius = '8px';
cacheStatsElement.style.backgroundColor = '#f9f9f9';
cacheStatsElement.style.fontFamily = 'monospace';
document.body.appendChild(cacheStatsElement);

// Verwendungsbeispiel
const observableCache = new ObservableCacheService();

// Cache-Statistiken überwachen
observableCache.stats$.subscribe(stats => {
  console.log('=== Cache-Statistiken ===');
  console.log(`Anzahl Einträge: ${stats.entries.length}`);
  console.log(`Gesamtgröße: ${(stats.totalSize / 1024).toFixed(2)} KB`);
  console.log(`Trefferquote: ${stats.hitRate.toFixed(1)}%`);

  // UI-Aktualisierung
  updateCacheStatsUI(stats, cacheStatsElement);
});

function updateCacheStatsUI(stats: CacheStats, element: HTMLElement): void {
  element.innerHTML = `
    <div style="margin-bottom: 15px;">
      <h3 style="margin: 0 0 10px 0; color: #333;">Cache-Statistiken</h3>
      <div style="padding: 10px; background-color: #fff; border-radius: 4px; margin: 5px 0;">
        <strong>Anzahl Einträge:</strong> ${stats.entries.length}
      </div>
      <div style="padding: 10px; background-color: #fff; border-radius: 4px; margin: 5px 0;">
        <strong>Gesamtgröße:</strong> ${(stats.totalSize / 1024).toFixed(2)} KB
      </div>
      <div style="padding: 10px; background-color: #fff; border-radius: 4px; margin: 5px 0;">
        <strong>Trefferquote:</strong> ${stats.hitRate.toFixed(1)}%
      </div>
    </div>
    <div>
      <h4 style="margin: 10px 0; color: #666;">Eintrags-Liste:</h4>
      ${stats.entries.map(e => `
        <div style="display: flex; justify-content: space-between; padding: 8px; margin: 5px 0; background-color: #fff; border-radius: 4px; border-left: 3px solid #2196F3;">
          <span style="font-weight: bold;">${e.key}</span>
          <span style="color: #2196F3;">${e.hits} Treffer</span>
        </div>
      `).join('')}
    </div>
  `;

  if (stats.entries.length === 0) {
    element.innerHTML = '<div style="padding: 20px; text-align: center; color: #999;">Keine Cache-Einträge vorhanden</div>';
  }
}
```

## Zusammenfassung

Durch die Beherrschung von Caching-Strategiemustern können Sie die Leistung und Benutzererfahrung erheblich verbessern.

> [!IMPORTANT] Wichtige Punkte
> - **shareReplay**: Optimal für grundlegendes Memory-Caching
> - **TTL**: Automatische Invalidierung alter Daten
> - **Manuelles Refresh**: Benutzergesteuerte Aktualisierung
> - **LocalStorage**: Dauerhafter Cache
> - **Offline-Unterstützung**: Cache-First-Strategie
> - **Überwachung**: Visualisierung von Trefferquote und Größe

> [!TIP] Best Practices
> - **Angemessenes TTL**: Ablaufzeit entsprechend der Datennatur festlegen
> - **Löschen bei Fehler**: Cache bei Fehlerauftritt verwerfen
> - **Größenverwaltung**: Obergrenze für Storage-Kapazität festlegen
> - **Verwendung von refCount**: Memory Leaks verhindern
> - **Cache-Schlüssel**: Eindeutige und verständliche Schlüssel verwenden

## Nächste Schritte

Nach der Beherrschung von Caching-Strategiemustern können Sie mit den folgenden Mustern fortfahren.

- [Echtzeit-Datenverarbeitung](./real-time-data.md) - Caching von Echtzeit-Daten
- [API-Aufrufe](./api-calls.md) - Caching von API-Antworten
- [UI-Ereignisverarbeitung](./ui-events.md) - Caching von Ereignisdaten
- Fehlerbehandlung in der Praxis (in Vorbereitung) - Behandlung von Cache-Fehlern

## Verwandte Abschnitte

- [Kapitel 2: Cold/Hot Observable](../observables/cold-and-hot-observables.md) - Details zu shareReplay
- [Kapitel 4: Operatoren](../operators/multicasting/shareReplay.md) - Verwendung von shareReplay
- [Kapitel 10: Anti-Muster](../anti-patterns/common-mistakes.md) - Fehlerhafte Verwendung von shareReplay

## Referenzen

- [RxJS Offiziell: shareReplay](https://rxjs.dev/api/operators/shareReplay) - Details zu shareReplay
- [MDN: Web Storage API](https://developer.mozilla.org/de/docs/Web/API/Web_Storage_API) - Verwendung von LocalStorage
- [Learn RxJS: Caching](https://www.learnrxjs.io/) - Praktische Beispiele für Caching-Muster
