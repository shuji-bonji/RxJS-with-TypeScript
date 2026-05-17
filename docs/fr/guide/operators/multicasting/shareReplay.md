---
description: "shareReplay est un opérateur de multidiffusion RxJS qui, en plus de la multidiffusion, met en tampon les valeurs passées et les fournit aux abonnés retardés. Idéal pour la mise en cache des réponses API, le partage des informations de configuration et la gestion de l'état. Prévention des fuites de mémoire avec les options refCount et windowTime, et mise en cache type-safe grâce à l'inférence de type TypeScript."
---

# shareReplay - Cache et partage

L'opérateur `shareReplay()` fournit la même multidiffusion que `share()`, mais en plus **mémorise un nombre spécifié de valeurs passées** et les rend disponibles aux abonnés qui se joignent plus tard.

Cela permet des cas d'utilisation plus avancés tels que la mise en cache des réponses de l'API et le partage de l'état.

[📘 Documentation officielle RxJS - `shareReplay()`](https://rxjs.dev/api/index/function/shareReplay)

## 🔰 Utilisation de base

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Utiliser shareReplay (taille du tampon 2)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source : ${value}`)),
  shareReplay(2) // Mettre en tampon les deux dernières valeurs
);

// Premier abonné
console.log('Observer 1 abonnement démarré');
source$.subscribe(value => console.log(`Observer 1 : ${value}`));

// Ajout d'un deuxième abonné après 3,5 secondes
setTimeout(() => {
  console.log('Observer 2 abonnement démarré - reçoit les deux valeurs les plus récentes');
  source$.subscribe(value => console.log(`Observer 2 : ${value}`));
}, 3500);
```

### Résultat de l'exécution

```
Observer 1 abonnement démarré
Source : 0
Observer 1 : 0
Source : 1
Observer 1 : 1
Source : 2
Observer 1 : 2
Source : 3
Observer 1 : 3
Observer 2 abonnement démarré - reçoit les deux valeurs les plus récentes
Observer 2 : 2  // ← Valeur passée mise en tampon
Observer 2 : 3  // ← Valeur passée mise en tampon
Source : 4
Observer 1 : 4
Observer 2 : 4
```

**Points importants** :
- Les abonnés retardés reçoivent également les valeurs passées mises en tampon immédiatement
- Les valeurs sont stockées pour la taille du tampon (deux dans cet exemple)

## 💡 Syntaxe de shareReplay()

```typescript
shareReplay(bufferSize?: number, windowTime?: number, scheduler?: SchedulerLike)
shareReplay(config: ShareReplayConfig)
```

### Paramètres

| Paramètre | Type | Description | Valeur par défaut |
|-----------|---|------|----------|
| `bufferSize` | `number` | Nombre de valeurs à mettre en tampon | `Infinity` |
| `windowTime` | `number` | Durée du tampon en millisecondes | `Infinity` |
| `scheduler` | `SchedulerLike` | Scheduler pour le contrôle du timing | - |

### Objet de configuration (RxJS 7+)

```typescript
interface ShareReplayConfig {
  bufferSize?: number;
  windowTime?: number;
  refCount?: boolean;  // Désabonner quand le nombre d'abonnés atteint 0
  scheduler?: SchedulerLike;
}
```

## 📊 Différence entre share et shareReplay

### Comportement de share()

```typescript
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source : ${value}`)),
  share()
);

source$.subscribe(value => console.log(`Observer 1 : ${value}`));

setTimeout(() => {
  console.log('Observer 2 abonnement démarré');
  source$.subscribe(value => console.log(`Observer 2 : ${value}`));
}, 1500);
```

**Résultat de l'exécution** :
```
Source : 0
Observer 1 : 0
Source : 1
Observer 1 : 1
Observer 2 abonnement démarré
Source : 2
Observer 1 : 2
Observer 2 : 2  // ← Les valeurs passées (0, 1) ne sont pas reçues
```

### Comportement de shareReplay()

```typescript
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source : ${value}`)),
  shareReplay(2) // Mettre en tampon les deux dernières valeurs
);

source$.subscribe(value => console.log(`Observer 1 : ${value}`));

setTimeout(() => {
  console.log('Observer 2 abonnement démarré');
  source$.subscribe(value => console.log(`Observer 2 : ${value}`));
}, 1500);
```

**Résultat de l'exécution** :
```
Source : 0
Observer 1 : 0
Source : 1
Observer 1 : 1
Observer 2 abonnement démarré
Observer 2 : 0  // ← Valeur passée mise en tampon
Observer 2 : 1  // ← Valeur passée mise en tampon
Source : 2
Observer 1 : 2
Observer 2 : 2
```

## 💼 Cas d'utilisation pratiques

### 1. Mettre en cache les réponses de l'API

```typescript
import { Observable } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, shareReplay, tap } from 'rxjs';

interface User {
  id: number;
  name: string;
  username: string;
  email: string;
}

class UserService {
  // Mettre en cache les informations utilisateur
  private userCache$ = ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
    tap(() => console.log('Requête API exécutée')),
    shareReplay({ bufferSize: 1, refCount: true }) // Cache la valeur la plus récente (libérée quand toutes les souscriptions sont annulées grâce à refCount)
  );

  getUser(): Observable<User> {
    return this.userCache$;
  }
}

const userService = new UserService();

// Premier composant
userService.getUser().subscribe(user => {
  console.log('Composant 1 :', user);
});

// 2 secondes plus tard, un autre composant
setTimeout(() => {
  userService.getUser().subscribe(user => {
    console.log('Composant 2 :', user); // ← Récupéré du cache, pas de requête API
  });
}, 2000);
```

**Résultat de l'exécution** :
```
Requête API exécutée
Composant 1 : { id: 1, name: "John" }
Composant 2 : { id: 1, name: "John" }  // ← Pas de requête API
```

### 2. Partage des informations de configuration

```typescript
import { of } from 'rxjs';
import { delay, shareReplay, tap } from 'rxjs';

// Obtenir la configuration de l'application (première exécution uniquement)
const appConfig$ = of({
  apiUrl: 'https://api.example.com',
  theme: 'dark',
  language: 'fr'
}).pipe(
  delay(1000), // Simuler le chargement
  tap(() => console.log('Configuration chargée')),
  shareReplay({ bufferSize: 1, refCount: true })
);

// Utilisation de la configuration dans plusieurs services
appConfig$.subscribe(config => console.log('Service A :', config.apiUrl));
appConfig$.subscribe(config => console.log('Service B :', config.theme));
appConfig$.subscribe(config => console.log('Service C :', config.language));
```

**Résultat de l'exécution** :
```
Configuration chargée
Service A : https://api.example.com
Service B : dark
Service C : fr
```

### 3. Cache limité dans le temps

```typescript
import { ajax } from 'rxjs/ajax';
import { shareReplay, tap } from 'rxjs';

// Cache pour 5 secondes seulement (en utilisant les données TODO comme exemple)
const todoData$ = ajax.getJSON('https://jsonplaceholder.typicode.com/todos/1').pipe(
  tap(() => console.log('Acquisition des données TODO')),
  shareReplay({
    bufferSize: 1,
    windowTime: 5000, // Valable pendant 5 secondes
    refCount: true    // Désabonner quand le nombre d'abonnés atteint 0
  })
);

// Premier abonnement
todoData$.subscribe(data => console.log('Récupération 1 :', data));

// Après 3 secondes (cache valide)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Récupération 2 :', data)); // Depuis le cache
}, 3000);

// Après 6 secondes (cache expiré)
setTimeout(() => {
  todoData$.subscribe(data => console.log('Récupération 3 :', data)); // Nouvelle requête
}, 6000);
```

## ⚠️ Attention aux fuites de mémoire

`shareReplay()` conserve les valeurs dans un tampon et peut provoquer des fuites de mémoire s'il n'est pas géré correctement.

### Code problématique

```typescript
// ❌ Risque de fuites de mémoire
const infiniteStream$ = interval(1000).pipe(
  shareReplay() // Taille du tampon non spécifiée = Infinity
);

// Ce flux continuera à accumuler des valeurs pour toujours
```

### Contre-mesures recommandées

```typescript
// ✅ Limiter la taille du tampon
const safeStream$ = interval(1000).pipe(
  shareReplay(1) // Ne conserver que la plus récente
);

// ✅ Utiliser refCount
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    refCount: true // Effacer le tampon quand le nombre d'abonnés atteint 0
  })
);

// ✅ Définir une limite de temps
const safeStream$ = interval(1000).pipe(
  shareReplay({
    bufferSize: 1,
    windowTime: 10000 // Expire dans 10 secondes
  })
);
```

## 🎯 Comment choisir la taille du tampon

| Taille du tampon | Cas d'utilisation | Exemple |
|--------------|-----------|---|
| `1` | Seul le dernier état est requis | Informations utilisateur actuelles, paramètres |
| `3-5` | Historique des derniers éléments requis | Historique du chat, historique des notifications |
| `Infinity` | Tout l'historique est requis | Journaux, piste d'audit (attention) |

## 🔄 Opérateurs associés

- **[share()](/fr/guide/operators/multicasting/share)** - Multidiffusion simple (pas de tampon)
- **[publish()](/fr/guide/subjects/multicasting)** - Contrôle de multidiffusion de bas niveau
- **[ReplaySubject](/fr/guide/subjects/types-of-subject)** - Subject sous-jacent de shareReplay

## Résumé

L'opérateur `shareReplay()` permet de :
- Mettre en tampon les valeurs passées et les fournir également aux abonnés retardés
- Idéal pour mettre en cache les réponses de l'API
- Les fuites de mémoire doivent être surveillées
- Utilisation sûre avec `refCount` et `windowTime`

Si vous avez besoin de partager ou de mettre en cache un état, `shareReplay()` est un outil très puissant, mais il est important de définir des tailles de tampon et des limites de temps appropriées.

## 🔗 Sections connexes

- **[Erreurs courantes et solutions](/fr/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用)** - Utilisation correcte de shareReplay et prévention des fuites de mémoire
- **[share()](/fr/guide/operators/multicasting/share)** - Multidiffusion simple
- **[ReplaySubject](/fr/guide/subjects/types-of-subject)** - Le Subject sous-jacent de shareReplay
