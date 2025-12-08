---
description: "Cet article détaille les différences entre Cold Observable et Hot Observable. Des exemples d'applications pratiques sont présentés, notamment l'indépendance du flux de données par abonnement, la conversion de Cold à Hot à l'aide de share et shareReplay, et la mise en cache des requêtes d'API."
---
# Cold Observable et Hot Observable

L'un des concepts clés de l'utilisation de RxJS est la distinction entre "Cold Observable" et "Hot Observable". Il est essentiel de comprendre cette distinction pour apprendre à utiliser efficacement Observable.

## Pourquoi il est important de comprendre Cold/Hot

Si vous ne comprenez pas la distinction Cold/Hot, vous serez confronté aux problèmes suivants :

- **Exécution en double involontaire** - Les appels API sont exécutés plusieurs fois
- **Fuites de mémoire** - Les abonnements ne sont pas correctement gérés
- **Problèmes de performances** - Des traitements inutiles sont répétés
- **Incohérences de données** - Les données attendues ne sont pas reçues

## Différences entre Cold et Hot (Tableau de comparaison)

Commençons par une vue d'ensemble.

| Élément de comparaison | Cold Observable | Hot Observable |
|----------|------------------|----------------|
| **Exécution sans abonnement** | Non exécuté (exécuté uniquement en cas d'abonnement) | Exécuté (flux de valeurs même sans abonnement) |
| **Timing de publication des données** | Démarre quand `subscribe()` est appelé | Démarre au timing de l'éditeur (indépendant de l'abonnement) |
| **Réutilisation de l'exécution** | Exécuté à nouveau à chaque fois | Partage du flux existant avec plusieurs abonnés |
| **Cohérence des données** | Chaque abonnement reçoit des valeurs indépendantes | Ne peut pas recevoir les valeurs passées si abonné en cours de flux |
| **Cas d'utilisation principaux** | Requêtes HTTP, traitement asynchrone | Événements UI, WebSocket, communication en temps réel |
| **Scénarios d'utilisation** | Quand chaque processus est indépendant | Partage d'état, diffusion d'événements |

**Critères :** Le traitement doit-il être réexécuté pour chaque abonné ? Ou le flux doit-il être partagé ?

## Critères de décision Cold vs. Hot

Les critères suivants peuvent être utilisés pour déterminer si un Observable est effectivement Cold ou Hot :

| Point de décision | Cold | Hot |
|-------------|------|-----|
| **La logique d'exécution est-elle réexécutée pour chaque abonnement ?** | ✅ Réexécution à chaque fois | ❌ Partage de l'exécution |
| **Les données circulent-elles avant l'abonnement ?** | ❌ Attend l'abonnement | ✅ Circule indépendamment de l'abonnement |
| **Plusieurs abonnements reçoivent-ils les mêmes données ?** | ❌ Données indépendantes | ✅ Partage les mêmes données |

### Moyens pratiques de distinction

Le test suivant permet de déterminer facilement :

```typescript
const observable$ = /* Observable à examiner */;

observable$.subscribe(/* abonnement 1 */);
observable$.subscribe(/* abonnement 2 */);

// ✅ Cold : console.log à l'intérieur de l'Observable s'exécute deux fois
//          (la logique d'exécution s'exécute à nouveau pour chaque abonnement)
// ✅ Hot :  console.log à l'intérieur de l'Observable s'exécute une seule fois
//          (l'exécution est partagée)
```

**Exemple concret :**

```typescript
import { Observable, Subject } from 'rxjs';

// Cold Observable
const cold$ = new Observable(subscriber => {
  console.log('Cold : Exécution démarrée');
  subscriber.next(Math.random());
});

cold$.subscribe(v => console.log('Abonnement 1:', v));
cold$.subscribe(v => console.log('Abonnement 2:', v));
// Sortie :
// Cold : Exécution démarrée  ← 1ère fois
// Abonnement 1: 0.123...
// Cold : Exécution démarrée  ← 2ème fois (réexécuté)
// Abonnement 2: 0.456...

// Hot Observable
const hot$ = new Subject();

hot$.subscribe(v => console.log('Abonnement 1:', v));
hot$.subscribe(v => console.log('Abonnement 2:', v));
hot$.next(1); // Données publiées une seule fois
// Sortie :
// Abonnement 1: 1
// Abonnement 2: 1  ← Partage les mêmes données
```

## Tableau de classification Cold/Hot par fonction de création

Ce tableau permet de classer Cold/Hot pour toutes les principales fonctions de création. Il vous permet de voir d'un coup d'œil quelle fonction produit quel Observable.

| Catégorie | Fonction de création | Cold/Hot | Notes |
|---------|-------------------|----------|------|
| **Création de base** | `of()` | ❄️ Cold | Re-publie les valeurs pour chaque abonnement |
| | `from()` | ❄️ Cold | Ré-exécute le tableau/Promise pour chaque abonnement |
| | `fromEvent()` | ❄️ Cold | Ajoute un listener indépendant pour chaque abonnement [^fromEvent] |
| | `interval()` | ❄️ Cold | Timer indépendant pour chaque abonnement |
| | `timer()` | ❄️ Cold | Timer indépendant pour chaque abonnement |
| **Génération en boucle** | `range()` | ❄️ Cold | Régénère la plage pour chaque abonnement |
| | `generate()` | ❄️ Cold | Ré-exécute la boucle pour chaque abonnement |
| **Communication HTTP** | `ajax()` | ❄️ Cold | Nouvelle requête HTTP pour chaque abonnement |
| | `fromFetch()` | ❄️ Cold | Nouvelle requête Fetch pour chaque abonnement |
| **Combinaison** | `concat()` | ❄️ Cold | Hérite de la nature des Observables sources [^combination] |
| | `merge()` | ❄️ Cold | Hérite de la nature des Observables sources [^combination] |
| | `combineLatest()` | ❄️ Cold | Hérite de la nature des Observables sources [^combination] |
| | `zip()` | ❄️ Cold | Hérite de la nature des Observables sources [^combination] |
| | `forkJoin()` | ❄️ Cold | Hérite de la nature des Observables sources [^combination] |
| **Sélection/Partition** | `race()` | ❄️ Cold | Hérite de la nature des Observables sources [^combination] |
| | `partition()` | ❄️ Cold | Hérite de la nature des Observables sources [^combination] |
| **Conditionnel** | `iif()` | ❄️ Cold | Hérite de la nature de l'Observable sélectionné conditionnellement |
| | `defer()` | ❄️ Cold | Exécute la fonction factory pour chaque abonnement |
| **Contrôle** | `scheduled()` | ❄️ Cold | Hérite de la nature de l'Observable source |
| | `using()` | ❄️ Cold | Crée une ressource pour chaque abonnement |
| **Famille Subject** | `new Subject()` | 🔥 Hot | Toujours Hot |
| | `new BehaviorSubject()` | 🔥 Hot | Toujours Hot |
| | `new ReplaySubject()` | 🔥 Hot | Toujours Hot |
| | `new AsyncSubject()` | 🔥 Hot | Toujours Hot |
| **WebSocket** | `webSocket()` | 🔥 Hot | Partage la connexion WebSocket |

[^fromEvent]: `fromEvent()` est Cold car il ajoute un listener d'événement indépendant pour chaque abonnement. Cependant, l'événement lui-même se produit indépendamment de l'abonnement, il est donc facilement mal compris comme Hot.

[^combination]: Les fonctions de création de combinaison sont Cold si l'Observable source est Cold, et Hot si elle est Hot. Habituellement, des Observables Cold sont combinés.

> [!IMPORTANT] Principe clé
> **Presque toutes les fonctions de création génèrent du Cold.**
> Seules les suivantes génèrent du Hot :
> - Famille Subject (Subject, BehaviorSubject, ReplaySubject, AsyncSubject)
> - webSocket()

## Cold Observable

### Caractéristiques

- **Un nouveau flux de données est créé à chaque fois qu'un abonnement est effectué**
- **La publication des données ne commence pas tant qu'il n'y a pas d'abonnement (exécution paresseuse)**
- **Tous les abonnés reçoivent toutes les données à partir du début de l'Observable**

Cold Observable crée un nouveau contexte d'exécution à chaque fois que vous vous abonnez.
Cela convient aux requêtes HTTP, au traitement asynchrone et à d'autres situations où un nouveau traitement est nécessaire à chaque fois.

### Exemple de code

```typescript
import { Observable } from 'rxjs';

// Exemple de Cold Observable
const cold$ = new Observable<number>(subscriber => {
  console.log('Création de source de données - nouvel abonnement');
  const randomValue = Math.random();
  subscriber.next(randomValue);
  subscriber.complete();
});

// Premier abonnement
console.log('--- Premier abonnement ---');
cold$.subscribe(value => console.log('Abonné 1:', value));

// Deuxième abonnement (des données différentes sont générées)
console.log('--- Deuxième abonnement ---');
cold$.subscribe(value => console.log('Abonné 2:', value));
```

#### Sortie
```sh
--- Premier abonnement ---
Création de source de données - nouvel abonnement
Abonné 1: 0.259632...
--- Deuxième abonnement ---
Création de source de données - nouvel abonnement  ← Réexécuté
Abonné 2: 0.744322...  ← Valeur différente
```

> [!TIP] Point important
> Chaque abonnement exécute "Création de source de données" et génère des valeurs différentes.

### Cold Observables courants (Comment les identifier)

Les Observables suivants sont généralement Cold :

```typescript
import { of, from, interval, timer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Fonctions de création
of(1, 2, 3)                    // Cold
from([1, 2, 3])                // Cold
from(fetch('/api/data'))       // Cold

// Opérateurs de temps
interval(1000)                 // Cold
timer(1000)                    // Cold

// Requêtes HTTP
ajax('/api/users')             // Cold
```

> [!TIP] Règle
> Les fonctions de création, les opérateurs de temps et les requêtes HTTP sont généralement Cold

## Hot Observable

### Caractéristiques

- **Émet des valeurs même sans abonnement (s'exécute avec ou sans abonnement)**
- **Ne reçoit des données qu'à partir du moment où l'abonnement est lancé**
- **Une source de données est partagée par plusieurs abonnés**

Avec Hot Observable, le timing de publication du flux est indépendant de l'abonnement, et les abonnés rejoignent le flux en cours.

### Exemple de code

```typescript
import { Subject } from 'rxjs';

// Exemple de Hot Observable (utilisant Subject)
const hot$ = new Subject<number>();

// Premier abonnement
console.log('--- Abonné 1 démarre ---');
hot$.subscribe(value => console.log('Abonné 1:', value));

// Publication de données
hot$.next(1);
hot$.next(2);

// Deuxième abonnement (abonnement tardif)
console.log('--- Abonné 2 démarre ---');
hot$.subscribe(value => console.log('Abonné 2:', value));

// Publication de plus de données
hot$.next(3);
hot$.next(4);

hot$.complete();
```

#### Sortie
```sh
--- Abonné 1 démarre ---
Abonné 1: 1
Abonné 1: 2
--- Abonné 2 démarre ---
Abonné 1: 3
Abonné 2: 3  ← Abonnement 2 rejoint à partir de 3 (ne peut pas recevoir 1, 2)
Abonné 1: 4
Abonné 2: 4
```

> [!TIP] Point important
> L'abonné 2 a rejoint en cours de flux et ne peut pas recevoir les valeurs passées (1, 2).

### Hot Observables courants (Comment les identifier)

Les Observables suivants sont toujours Hot :

```typescript
import { Subject, BehaviorSubject, ReplaySubject } from 'rxjs';
import { webSocket } from 'rxjs/webSocket';

// Famille Subject (toujours Hot)
new Subject()                  // Hot
new BehaviorSubject(0)         // Hot
new ReplaySubject(1)           // Hot

// WebSocket (toujours Hot)
webSocket('ws://localhost:8080') // Hot
```

> [!TIP] Règle
> **Seuls la famille Subject et webSocket() génèrent du Hot**

> [!WARNING] fromEvent() est Cold
> `fromEvent(button, 'click')` est souvent considéré à tort comme Hot, mais il est en fait **Cold**. Il ajoute un listener d'événement indépendant pour chaque abonnement. L'événement lui-même se produit indépendamment de l'abonnement, mais chaque abonné a un listener indépendant.

## Comment convertir un Cold Observable en Hot

En RxJS, les principaux moyens de convertir un Cold Observable en Hot sont les suivants :

- `share()` - Conversion simple en hot (recommandé)
- `shareReplay()` - Met en cache les valeurs passées et convertit en hot
- ~~`multicast()`~~ - Déprécié (déprécié dans RxJS v7, supprimé dans v8)

### Opérateur share()

`share()` est la façon la plus courante de convertir un Cold Observable en Hot Observable.

```typescript
import { interval } from 'rxjs';
import { share, take } from 'rxjs';

// Simuler un appel HTTP
const makeHttpRequest = () => {
  console.log('Appel HTTP exécuté !');
  return interval(1000).pipe(take(3));
};

// ❌ Cold Observable (pas de partage)
const cold$ = makeHttpRequest();

cold$.subscribe(val => console.log('Abonné 1:', val));
cold$.subscribe(val => console.log('Abonné 2:', val));
// → Appel HTTP exécuté deux fois

// ✅ Hot Observable (utilisant share)
const shared$ = makeHttpRequest().pipe(share());

shared$.subscribe(val => console.log('Abonné partagé 1:', val));
shared$.subscribe(val => console.log('Abonné partagé 2:', val));
// → Appel HTTP exécuté une seule fois, le résultat est partagé
```

**Sortie (Cold) :**
```sh
Appel HTTP exécuté !  ← 1ère fois
Abonné 1: 0
Appel HTTP exécuté !  ← 2ème fois (doublon !)
Abonné 2: 0
...
```

**Sortie (Hot) :**
```sh
Appel HTTP exécuté !  ← Une seule fois
Abonné partagé 1: 0
Abonné partagé 2: 0  ← Partage le même flux
...
```

> [!NOTE] Cas d'utilisation
> - Utiliser les mêmes résultats d'API dans plusieurs composants
> - Éviter les effets secondaires en double (par exemple, les appels HTTP)

### Opérateur shareReplay()

`shareReplay()` est une extension de `share()` qui **met en cache** les valeurs passées et les rediffuse aux nouveaux abonnés.

```typescript
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

const request$ = interval(1000).pipe(
  take(3),
  shareReplay(2)  // Mettre en cache les 2 dernières valeurs
);

// Premier abonnement
request$.subscribe(val => console.log('Abonné 1:', val));

// Deuxième abonnement après 3,5 secondes (après la fin du flux)
setTimeout(() => {
  console.log('--- Abonné 2 démarre (après la fin) ---');
  request$.subscribe(val => console.log('Abonné 2:', val));
}, 3500);
```

#### Sortie
```sh
Abonné 1: 0
Abonné 1: 1
Abonné 1: 2
--- Abonné 2 démarre (après la fin) ---
Abonné 2: 1  ← Valeur mise en cache (2 dernières)
Abonné 2: 2  ← Valeur mise en cache
```

> [!NOTE] Cas d'utilisation
> - Mise en cache des résultats d'API
> - Partager l'état initial (ne mettre en cache que le dernier)
> - Fournir des données historiques aux abonnés tardifs

> [!WARNING] Notes sur shareReplay
> `shareReplay()` continue à conserver le cache même lorsque les abonnements passent à 0, ce qui peut provoquer des fuites de mémoire. Voir [Chapitre 10 : Mauvaise utilisation de shareReplay](/fr/guide/anti-patterns/common-mistakes#4-sharereplay-misuse) pour plus d'informations.

### À propos de multicast()

> [!NOTE]
> `multicast()` est flexible, mais a été déprécié dans RxJS v7 et supprimé dans v8. Utilisez `share()` ou `shareReplay()` maintenant. Voir [description de l'opérateur share()](/fr/guide/operators/multicasting/share) pour plus d'informations.

## Exemple pratique : Service de cache d'API

Modèle courant dans les applications réelles : plusieurs composants ont besoin des mêmes données d'API.

```typescript
import { Observable, of, throwError } from 'rxjs';
import { catchError, shareReplay, delay, tap } from 'rxjs';

// Service de cache simple
class UserService {
  private cache$: Observable<User[]> | null = null;

  getUsers(): Observable<User[]> {
    // Retourner le cache s'il existe
    if (this.cache$) {
      console.log('Retour depuis le cache');
      return this.cache$;
    }

    // Créer une nouvelle requête et mettre en cache
    console.log('Exécution d\'une nouvelle requête');
    this.cache$ = this.fetchUsersFromAPI().pipe(
      catchError(err => {
        this.cache$ = null;  // Effacer le cache en cas d'erreur
        return throwError(() => err);
      }),
      shareReplay(1)  // Mettre en cache le dernier résultat
    );

    return this.cache$;
  }

  private fetchUsersFromAPI(): Observable<User[]> {
    // Simuler une requête API réelle
    return of([
      { id: 1, name: 'Taro Yamada' },
      { id: 2, name: 'Hanako Sato' }
    ]).pipe(
      delay(1000),
      tap(() => console.log('Données reçues de l\'API'))
    );
  }

  clearCache(): void {
    this.cache$ = null;
    console.log('Cache effacé');
  }
}

interface User {
  id: number;
  name: string;
}

// Exemple d'utilisation
const userService = new UserService();

// Composant 1 : Demande de données
userService.getUsers().subscribe(users =>
  console.log('Composant 1:', users)
);

// Composant 2 : Demande de données après 2 secondes
setTimeout(() => {
  userService.getUsers().subscribe(users =>
    console.log('Composant 2:', users)
  );
}, 2000);

// Effacer le cache et demander à nouveau
setTimeout(() => {
  userService.clearCache();
  userService.getUsers().subscribe(users =>
    console.log('Composant 3:', users)
  );
}, 4000);
```

#### Sortie
```sh
Exécution d'une nouvelle requête
Données reçues de l'API
Composant 1: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
Retour depuis le cache  ← Pas d'appel API
Composant 2: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
Cache effacé
Exécution d'une nouvelle requête  ← Nouvel appel API
Données reçues de l'API
Composant 3: [{id: 1, name: 'Taro Yamada'}, {id: 2, name: 'Hanako Sato'}]
```

**Points :**
- Mettre en cache la dernière réponse avec `shareReplay(1)`
- Plusieurs composants partagent les données (un seul appel API)
- Abandonner le cache de manière appropriée en cas d'erreur ou d'effacement

## Quand utiliser

<div class="comparison-cards">

::: tip Cold
#### Quand utiliser
- Quand chaque abonné a besoin de son propre ensemble de données
- Quand il s'agit de représenter un processus ou une action qui vient de démarrer
- Quand les effets secondaires en double ne posent pas de problème

#### Exemples
- Envoyer une nouvelle requête POST pour chaque soumission de formulaire
- Timer différent pour chaque utilisateur
- Exécuter un calcul indépendant pour chaque abonnement
:::

::: tip Hot
#### Quand utiliser
- Quand vous partagez des données entre plusieurs composants
- Quand vous voulez économiser des ressources (par ex., réduire le nombre d'appels HTTP)
- Quand vous représentez des flux d'événements
- Gestion d'état ou communication inter-services

#### Exemples
- Informations de configuration partagées dans l'application
- État de connexion de l'utilisateur
- Messages en temps réel (WebSocket)
- Événements DOM (clic, défilement, etc.)
:::

</div>

## Résumé

Comprendre et utiliser correctement Cold Observable et Hot Observable est une compétence importante pour construire des applications RxJS efficaces.

::: tip Points clés
- **Cold Observable** : Un flux qui ne commence à s'exécuter qu'après avoir été souscrit (exécution indépendante par abonnement)
- **Hot Observable** : Partage un flux déjà en cours d'exécution (même exécution pour plusieurs abonnements)
- **share()** : Le moyen le plus simple de convertir Cold en Hot
- **shareReplay()** : Met en cache les valeurs passées et convertit en Hot (utile pour partager les résultats d'API)
:::

::: tip Critères de décision de conception
- Avez-vous besoin de partager des données entre plusieurs abonnés ?
- Est-il nécessaire de mettre en cache les valeurs passées et de les fournir aux nouveaux abonnés ?
- Comment les effets secondaires en double (par ex., les requêtes HTTP) seront-ils gérés ?
:::

Sur la base de ces considérations, sélectionner le type d'Observable et l'opérateur appropriés vous aidera à construire une application réactive efficace et robuste.

## Sections connexes

- **[Opérateur share()](/fr/guide/operators/multicasting/share)** - Explication détaillée de share()
- **[Mauvaise utilisation de shareReplay](/fr/guide/anti-patterns/common-mistakes#4-sharereplay-misuse)** - Erreurs courantes et solutions
- **[Subject](/fr/guide/subjects/what-is-subject)** - Comprendre les Subjects Hot

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>
