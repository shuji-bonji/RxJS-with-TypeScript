---
description: "Stratégies de mise en cache pratiques utilisant RxJS. Mise en cache avec shareReplay, TTL (Time To Live), invalidation de cache, cache multi-niveaux, stratégies offline-first, persistance avec localStorage, gestion des erreurs et stratégies de mise à jour sont expliquées avec des exemples d'implémentation concrets en TypeScript pour une utilisation professionnelle immédiate."
---

# Stratégies de mise en cache

La mise en cache est un élément clé pour améliorer les performances et l'expérience utilisateur des applications web modernes. En utilisant RxJS, vous pouvez mettre en œuvre efficacement des stratégies de mise en cache sophistiquées.

Cet article explique les modèles concrets de mise en cache nécessaires dans des situations réelles, de la mise en cache simple avec `shareReplay` aux stratégies avancées avec TTL et invalidation.

## Ce que vous apprendrez dans cet article

- Mise en cache de base avec shareReplay
- Mise en cache avec TTL (Time To Live)
- Invalidation et actualisation du cache
- Mise en cache multi-niveaux (mémoire + localStorage)
- Stratégies offline-first
- Gestion des erreurs de cache
- Stratégies de mise à jour (revalidation, prefetch)

> [!TIP] Prérequis
> Cet article suppose une connaissance de [Chapitre 2: Cold/Hot Observable](../observables/cold-and-hot-observables.md) et [Chapitre 4: Opérateurs](../operators/index.md). La compréhension de `shareReplay`, `share`, `switchMap` est particulièrement importante.

## Mise en cache de base avec shareReplay

### Problème : Éviter les appels API redondants

Lors de plusieurs abonnements au même Observable, des requêtes API redondantes sont envoyées à chaque fois. Nous voulons mettre en cache le résultat et le réutiliser.

### Solution : Utiliser shareReplay

`shareReplay` convertit un Cold Observable en Hot Observable et met en cache le résultat. Les abonnés ultérieurs reçoivent les valeurs mises en cache.

**Flux d'exécution :**

```
Sans shareReplay (Cold Observable) :
Abonné 1 → API appel 1
Abonné 2 → API appel 2
Abonné 3 → API appel 3
※ 3 appels API

Avec shareReplay (Hot Observable) :
Abonné 1 → API appel (mise en cache)
Abonné 2 → Depuis le cache
Abonné 3 → Depuis le cache
※ 1 seul appel API
```

```typescript
import { from, Observable, shareReplay } from 'rxjs';

// API JSONPlaceholder - Type User
// https://jsonplaceholder.typicode.com/users
interface User {
  id: number;
  name: string;
  username: string;
  email: string;
  address: {
    street: string;
    suite: string;
    city: string;
    zipcode: string;
    geo: {
      lat: string;
      lng: string;
    };
  };
  phone: string;
  website: string;
  company: {
    name: string;
    catchPhrase: string;
    bs: string;
  };
}

// ❌ Sans mise en cache : Appel API à chaque abonnement
function getUsersWithoutCache(): Observable<User[]> {
  console.log('Appel API');
  return from(
    fetch('https://jsonplaceholder.typicode.com/users')
      .then(response => {
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        return response.json();
      })
  );
}

// ✅ Avec mise en cache : Mise en cache du résultat de l'appel API
function getUsersWithCache(): Observable<User[]> {
  console.log('Configuration du cache');
  return from(
    fetch('https://jsonplaceholder.typicode.com/users')
      .then(response => {
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        return response.json();
      })
  ).pipe(
    shareReplay({
      bufferSize: 1,      // Nombre de valeurs à mettre en cache
      refCount: false      // Maintenir le cache même après désabonnement
    })
  );
}

// Exemple sans mise en cache
const withoutCache$ = getUsersWithoutCache();

withoutCache$.subscribe(users => console.log('Abonné 1:', users.length)); // Appel API 1
withoutCache$.subscribe(users => console.log('Abonné 2:', users.length)); // Appel API 2
withoutCache$.subscribe(users => console.log('Abonné 3:', users.length)); // Appel API 3

// Exemple avec mise en cache
const withCache$ = getUsersWithCache();

withCache$.subscribe(users => console.log('Abonné 1 (cache):', users.length)); // Appel API
withCache$.subscribe(users => console.log('Abonné 2 (cache):', users.length)); // Depuis le cache
withCache$.subscribe(users => console.log('Abonné 3 (cache):', users.length)); // Depuis le cache
```

> [!NOTE] Options de shareReplay
> - **bufferSize**: Nombre de valeurs à mettre en cache (généralement 1)
> - **refCount**:
>   - `true` - Libérer le cache quand il n'y a plus d'abonnés (par défaut)
>   - `false` - Maintenir le cache en permanence

### refCount true vs false

```typescript
import { shareReplay } from 'rxjs';
// refCount: true - Libération automatique du cache
const autoRelease$ = getUsersAPI().pipe(
  shareReplay({ bufferSize: 1, refCount: true })
);

// Abonnement 1
const sub1 = autoRelease$.subscribe(/*...*/);
sub1.unsubscribe(); // Le cache est libéré après le désabonnement

// L'abonnement suivant refait l'appel API
autoRelease$.subscribe(/*...*/); // Nouvel appel API

// refCount: false - Maintien permanent du cache
const permanent$ = getUsersAPI().pipe(
  shareReplay({ bufferSize: 1, refCount: false })
);

// Abonnement 1
const sub2 = permanent$.subscribe(/*...*/);
sub2.unsubscribe(); // Le cache reste

// L'abonnement suivant utilise le cache
permanent$.subscribe(/*...*/); // Depuis le cache
```

> [!TIP] Critères de sélection de refCount
> - **refCount: true** - Données qui changent fréquemment, libération de mémoire
> - **refCount: false** - Données statiques, cache de longue durée

## Mise en cache avec TTL (Time To Live)

### Problème : Les données mises en cache deviennent obsolètes

Avec `shareReplay`, le cache est permanent, et les anciennes données peuvent être retournées même si elles changent côté serveur. Nous voulons définir une durée de vie du cache (TTL) et rafraîchir périodiquement les données.

### Solution : Mise en œuvre du cache TTL

**Fonctionnement du TTL :**
1. Lors de la première requête, les données sont récupérées depuis l'API
2. Les requêtes suivantes retournent les données mises en cache (si dans la période de validité)
3. Après expiration du TTL, un nouvel appel API est effectué

**Exemple concret de TTL :**
- Données de profil utilisateur : 5 minutes
- Données de produit : 10 minutes
- Flux d'actualités : 30 secondes
- Données statiques : 1 heure ou plus

```typescript
import { defer, of, tap, shareReplay, Observable, timer, switchMap } from 'rxjs';

interface CachedData<T> {
  data: T;
  timestamp: number;
  expiresAt: number;
}

class TTLCache<T> {
  private cache = new Map<string, CachedData<T>>();

  /**
   * Obtenir des données avec TTL
   * @param key Clé de cache
   * @param fetchFn Fonction de récupération des données
   * @param ttl Durée de vie (ms)
   */
  get(
    key: string,
    fetchFn: () => Observable<T>,
    ttl: number = 5 * 60 * 1000 // Défaut 5 minutes
  ): Observable<T> {
    return defer(() => {
      const cached = this.cache.get(key);
      const now = Date.now();

      // Retourner le cache s'il est valide
      if (cached && cached.expiresAt > now) {
        console.log(`Hit du cache: ${key} (reste ${Math.floor((cached.expiresAt - now) / 1000)}s)`);
        return of(cached.data);
      }

      // Rafraîchir le cache s'il est expiré
      console.log(`Miss du cache: ${key} - Récupération de nouvelles données`);
      return fetchFn().pipe(
        tap(data => {
          this.cache.set(key, {
            data,
            timestamp: now,
            expiresAt: now + ttl
          });
          console.log(`Cache mis à jour: ${key} (TTL: ${ttl / 1000}s)`);
        }),
        shareReplay({ bufferSize: 1, refCount: false })
      );
    });
  }

  /**
   * Invalider le cache
   */
  invalidate(key: string): void {
    this.cache.delete(key);
    console.log(`Cache invalidé: ${key}`);
  }

  /**
   * Effacer tout le cache
   */
  clear(): void {
    this.cache.clear();
    console.log('Tout le cache a été effacé');
  }

  /**
   * Vérifier si le cache existe et est valide
   */
  has(key: string): boolean {
    const cached = this.cache.get(key);
    if (!cached) return false;

    const now = Date.now();
    return cached.expiresAt > now;
  }
}

// Type User de l'API JSONPlaceholder
interface User {
  id: number;
  name: string;
  username: string;
  email: string;
}

// Exemple d'utilisation
const cache = new TTLCache<User[]>();

function fetchUsers(): Observable<User[]> {
  return from(
    fetch('https://jsonplaceholder.typicode.com/users')
      .then(response => response.json())
  );
}

// Première requête - Appel API
cache.get('users', fetchUsers, 10000).subscribe(users => {
  console.log('Utilisateurs:', users.length); // Appel API
});

// Requête immédiate suivante - Depuis le cache
setTimeout(() => {
  cache.get('users', fetchUsers, 10000).subscribe(users => {
    console.log('Utilisateurs (cache):', users.length); // Depuis le cache
  });
}, 1000);

// Après 11 secondes - Nouvel appel API (TTL expiré)
setTimeout(() => {
  cache.get('users', fetchUsers, 10000).subscribe(users => {
    console.log('Utilisateurs (rafraîchi):', users.length); // Appel API
  });
}, 11000);
```

> [!IMPORTANT] Conception du TTL
> - **Trop court** : Charge serveur accrue, diminution des performances
> - **Trop long** : Les utilisateurs voient des données obsolètes
> - **Approprié** : Équilibre entre la fraîcheur des données et la charge serveur

### Rafraîchissement automatique du cache (Revalidation)

**Revalidation :**
- Les données mises en cache sont d'abord retournées immédiatement (réponse rapide)
- Les dernières données sont récupérées en arrière-plan
- Le cache est mis à jour une fois les données fraîches obtenues

**Avantages :**
- Expérience utilisateur rapide (pas d'attente pour les données mises en cache)
- Toujours récupérer les dernières données en arrière-plan
- Utilisé par SWR (stale-while-revalidate)

```typescript
import { merge, of, tap, shareReplay } from 'rxjs';
/**
 * Stratégie SWR (Stale-While-Revalidate)
 * Retourner d'abord les données mises en cache, puis actualiser en arrière-plan
 */
function getWithRevalidation<T>(
  key: string,
  fetchFn: () => Observable<T>,
  ttl: number = 60000
): Observable<T> {
  const cached = cache.get(key);
  const now = Date.now();

  // Retourner d'abord le cache s'il existe
  if (cached && cached.expiresAt > now) {
    console.log('Retour des données mises en cache et revalidation');

    return merge(
      of(cached.data), // Retourner immédiatement les données mises en cache
      fetchFn().pipe(   // Récupérer de nouvelles données en arrière-plan
        tap(data => {
          cache.set(key, {
            data,
            timestamp: now,
            expiresAt: now + ttl
          });
          console.log('Cache revalidé avec de nouvelles données');
        })
      )
    );
  }

  // Si pas de cache ou expiré, récupération normale
  return fetchFn().pipe(
    tap(data => {
      cache.set(key, {
        data,
        timestamp: now,
        expiresAt: now + ttl
      });
    }),
    shareReplay({ bufferSize: 1, refCount: false })
  );
}

// Exemple d'utilisation
getWithRevalidation('users', fetchUsers).subscribe(users => {
  console.log('Utilisateurs:', users.length); // Données mises en cache → Nouvelles données
});
```

> [!TIP] Cas d'utilisation de la Revalidation
> - Flux d'actualités, chronologie des réseaux sociaux
> - Catalogues de produits, listes d'articles
> - Profils utilisateur
> - → Scénarios où afficher d'abord le cache puis mettre à jour est préféré

## Invalidation et actualisation du cache

### Problème : Maintenir la cohérence des données lors de mutations

Lors de la création ou de la mise à jour de données (POST/PUT/PATCH), le cache existant devient obsolète. Il est nécessaire d'invalider le cache et de le rafraîchir.

### Solution : Implémentation d'une stratégie d'invalidation

**Modèles d'invalidation de cache :**
1. **Invalidation complète** - Invalider tout le cache
2. **Invalidation par clé** - Invalider uniquement les clés spécifiques
3. **Invalidation par motif** - Invalider les clés correspondant à un motif

**Méthodes d'actualisation après invalidation :**
- **Rafraîchissement immédiat** - Récupérer immédiatement de nouvelles données
- **Rafraîchissement paresseux** - Rafraîchir lors de la prochaine requête

```typescript
import { tap, catchError, switchMap, from } from 'rxjs';

interface Post {
  userId: number;
  id: number;
  title: string;
  body: string;
}

class CacheInvalidationService {
  private cache = new TTLCache<any>();

  /**
   * Créer une nouvelle publication (POST)
   * Invalider le cache des publications après création
   */
  createPost(post: Omit<Post, 'id'>): Observable<Post> {
    return from(
      fetch('https://jsonplaceholder.typicode.com/posts', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(post)
      }).then(r => r.json())
    ).pipe(
      tap(() => {
        // Invalider le cache des listes de publications
        this.cache.invalidate('posts');
        console.log('Cache des publications invalidé après création');
      }),
      // Récupérer immédiatement de nouvelles données (rafraîchissement immédiat)
      switchMap(newPost => {
        return this.cache.get('posts', () => this.fetchPosts());
      })
    );
  }

  /**
   * Mettre à jour une publication (PUT)
   * Invalider le cache de la publication spécifique
   */
  updatePost(id: number, post: Partial<Post>): Observable<Post> {
    return from(
      fetch(`https://jsonplaceholder.typicode.com/posts/${id}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(post)
      }).then(r => r.json())
    ).pipe(
      tap(() => {
        // Invalider à la fois la publication individuelle et la liste
        this.cache.invalidate(`post-${id}`);
        this.cache.invalidate('posts');
        console.log(`Cache de la publication ${id} invalidé`);
      })
    );
  }

  /**
   * Supprimer une publication (DELETE)
   */
  deletePost(id: number): Observable<void> {
    return from(
      fetch(`https://jsonplaceholder.typicode.com/posts/${id}`, {
        method: 'DELETE'
      }).then(() => {})
    ).pipe(
      tap(() => {
        this.cache.invalidate(`post-${id}`);
        this.cache.invalidate('posts');
        console.log(`Cache de la publication ${id} supprimé`);
      })
    );
  }

  /**
   * Obtenir la liste des publications
   */
  private fetchPosts(): Observable<Post[]> {
    return from(
      fetch('https://jsonplaceholder.typicode.com/posts')
        .then(r => r.json())
    );
  }

  /**
   * Obtenir une publication individuelle
   */
  getPost(id: number): Observable<Post> {
    return this.cache.get(
      `post-${id}`,
      () => from(
        fetch(`https://jsonplaceholder.typicode.com/posts/${id}`)
          .then(r => r.json())
      ),
      300000 // TTL 5 minutes
    );
  }
}

// Exemple d'utilisation
const service = new CacheInvalidationService();

// Créer une nouvelle publication
service.createPost({
  userId: 1,
  title: 'Nouvelle publication',
  body: 'Contenu de la nouvelle publication'
}).subscribe(post => {
  console.log('Publication créée:', post);
  // Le cache des publications est automatiquement invalidé et rafraîchi
});
```

> [!WARNING] Points d'attention pour l'invalidation du cache
> - **Éviter les invalidations excessives** : Une invalidation trop fréquente diminue l'efficacité du cache
> - **Considération de la granularité** : Équilibre entre invalidation complète et par clé
> - **Cohérence des données** : S'assurer d'invalider tous les caches associés

### Invalidation automatique par tags

**Système de tags de cache :**
- Associer plusieurs tags à une clé de cache
- Invalider tous les caches avec un tag spécifique
- Gestion fine de l'invalidation

**Exemples de tags :**
- `user:123` - Données de l'utilisateur avec ID 123
- `posts:user:123` - Publications de l'utilisateur 123
- `comments:post:456` - Commentaires de la publication 456

**Avantages :**
- Invalidation groupée facile des caches associés
- Relations de données claires
- Invalidation fine

```typescript
import { Observable } from 'rxjs';
interface TaggedCache<T> {
  data: T;
  timestamp: number;
  expiresAt: number;
  tags: Set<string>;
}

class TaggedCacheService {
  private cache = new Map<string, TaggedCache<any>>();
  private tagIndex = new Map<string, Set<string>>(); // tag → keys

  /**
   * Obtenir des données avec tags
   */
  get<T>(
    key: string,
    fetchFn: () => Observable<T>,
    tags: string[],
    ttl: number = 300000
  ): Observable<T> {
    const cached = this.cache.get(key);
    const now = Date.now();

    if (cached && cached.expiresAt > now) {
      console.log(`Hit du cache (tags: ${Array.from(cached.tags).join(', ')})`);
      return of(cached.data);
    }

    return fetchFn().pipe(
      tap(data => {
        const tagSet = new Set(tags);

        this.cache.set(key, {
          data,
          timestamp: now,
          expiresAt: now + ttl,
          tags: tagSet
        });

        // Mise à jour de l'index des tags
        tags.forEach(tag => {
          if (!this.tagIndex.has(tag)) {
            this.tagIndex.set(tag, new Set());
          }
          this.tagIndex.get(tag)!.add(key);
        });

        console.log(`Cache défini avec tags: ${tags.join(', ')}`);
      })
    );
  }

  /**
   * Invalider par tag
   */
  invalidateByTag(tag: string): void {
    const keys = this.tagIndex.get(tag);
    if (!keys) return;

    keys.forEach(key => {
      this.cache.delete(key);
    });

    this.tagIndex.delete(tag);
    console.log(`Cache invalidé par tag: ${tag} (${keys.size} entrées)`);
  }

  /**
   * Invalider par plusieurs tags (AND)
   */
  invalidateByTags(tags: string[]): void {
    tags.forEach(tag => this.invalidateByTag(tag));
  }
}

// Exemple d'utilisation
const taggedCache = new TaggedCacheService();

// Mettre en cache avec tags
taggedCache.get(
  'user-123-posts',
  fetchUserPosts,
  ['user:123', 'posts'], // Associer plusieurs tags
  600000
);

// Invalider tous les caches liés à l'utilisateur 123
taggedCache.invalidateByTag('user:123');

// Invalider tous les caches de publications
taggedCache.invalidateByTag('posts');
```

## Mise en cache multi-niveaux (Mémoire + localStorage)

### Problème : Fournir un cache rapide et persistant

**Limitations du cache en mémoire :**
- Perdu au rechargement de la page
- Ne fonctionne pas entre onglets
- Perdu à la fermeture du navigateur

**Solution :**
Combiner cache en mémoire (rapide) et localStorage (persistant) pour une stratégie de cache multi-niveaux.

**Hiérarchie du cache :**
1. **Cache L1 (Mémoire)** - Très rapide, volatil
2. **Cache L2 (localStorage)** - Rapide, persistant
3. **Cache L3 (IndexedDB)** - Grand, persistant (optionnel)
4. **Origine (API)** - Le plus lent, toujours à jour

```typescript
import { defer, of, tap, Observable, from } from 'rxjs';
interface CachedItem<T> {
  data: T;
  timestamp: number;
  expiresAt: number;
}

class MultiLevelCache<T> {
  private memoryCache = new Map<string, CachedItem<T>>();

  /**
   * Obtenir des données (vérifier Mémoire → localStorage → API)
   */
  get(
    key: string,
    fetchFn: () => Observable<T>,
    ttl: number = 300000
  ): Observable<T> {
    return defer(() => {
      const now = Date.now();

      // L1: Vérifier le cache mémoire
      const memCached = this.memoryCache.get(key);
      if (memCached && memCached.expiresAt > now) {
        console.log('Hit du cache L1 (mémoire)');
        return of(memCached.data);
      }

      // L2: Vérifier localStorage
      const localCached = this.getFromLocalStorage(key);
      if (localCached && localCached.expiresAt > now) {
        console.log('Hit du cache L2 (localStorage)');
        // Promouvoir vers le cache mémoire
        this.memoryCache.set(key, localCached);
        return of(localCached.data);
      }

      // L3: Récupérer depuis l'API
      console.log('Miss du cache - Récupération depuis l\'API');
      return fetchFn().pipe(
        tap(data => {
          const cacheItem: CachedItem<T> = {
            data,
            timestamp: now,
            expiresAt: now + ttl
          };

          // Définir à la fois en mémoire et localStorage
          this.memoryCache.set(key, cacheItem);
          this.setToLocalStorage(key, cacheItem);
        })
      );
    });
  }

  /**
   * Obtenir depuis localStorage
   */
  private getFromLocalStorage(key: string): CachedItem<T> | null {
    try {
      const item = localStorage.getItem(`cache:${key}`);
      if (!item) return null;

      return JSON.parse(item) as CachedItem<T>;
    } catch (error) {
      console.error('Erreur de lecture du localStorage:', error);
      return null;
    }
  }

  /**
   * Définir dans localStorage
   */
  private setToLocalStorage(key: string, item: CachedItem<T>): void {
    try {
      localStorage.setItem(`cache:${key}`, JSON.stringify(item));
    } catch (error) {
      console.error('Erreur d\'écriture du localStorage:', error);
      // Ignorer les erreurs de quota
    }
  }

  /**
   * Invalider le cache
   */
  invalidate(key: string): void {
    this.memoryCache.delete(key);
    localStorage.removeItem(`cache:${key}`);
    console.log(`Cache invalidé (tous les niveaux): ${key}`);
  }

  /**
   * Effacer tout le cache
   */
  clear(): void {
    this.memoryCache.clear();

    // Supprimer toutes les clés commençant par "cache:"
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key?.startsWith('cache:')) {
        localStorage.removeItem(key);
      }
    }

    console.log('Tout le cache a été effacé (tous les niveaux)');
  }
}

// Exemple d'utilisation
const multiCache = new MultiLevelCache<User[]>();

// Première requête - Depuis l'API
multiCache.get('users', fetchUsers, 600000).subscribe(users => {
  console.log('Utilisateurs:', users.length); // Depuis l'API
});

// Requête immédiate suivante - Depuis le cache mémoire
multiCache.get('users', fetchUsers, 600000).subscribe(users => {
  console.log('Utilisateurs:', users.length); // Depuis le cache L1
});

// Après rechargement de la page - Depuis localStorage
// (Le cache mémoire est vide mais localStorage existe)
multiCache.get('users', fetchUsers, 600000).subscribe(users => {
  console.log('Utilisateurs:', users.length); // Depuis le cache L2
});
```

> [!TIP] Stratégie de cache multi-niveaux
> **Avantages :**
> - Cache rapide avec la mémoire
> - Persistance avec localStorage
> - Expérience utilisateur cohérente même après rechargement de la page
>
> **Points d'attention :**
> - Limite de quota de localStorage (généralement 5-10MB)
> - Gestion des erreurs lors de l'écriture
> - Nettoyage du cache expiré

## Stratégie offline-first

### Problème : Fournir un cache même hors ligne

Les applications web progressives (PWA) doivent fonctionner même lorsque le réseau est indisponible. Une stratégie offline-first est nécessaire.

**Modèle offline-first :**
1. **Retourner d'abord le cache** (réponse instantanée)
2. **Récupérer depuis le réseau en arrière-plan** (mise à jour)
3. **Stocker les nouvelles données dans le cache**

**Avantages :**
- Fonctionne même hors ligne
- Réponse rapide
- Mise à jour automatique quand le réseau est disponible

```typescript
import { merge, of, from, catchError, tap, shareReplay } from 'rxjs';
class OfflineFirstCache<T> {
  private memoryCache = new Map<string, CachedItem<T>>();

  /**
   * Stratégie offline-first
   * Retourner d'abord le cache, puis actualiser depuis le réseau
   */
  get(
    key: string,
    fetchFn: () => Observable<T>,
    ttl: number = 600000
  ): Observable<T> {
    const now = Date.now();

    // Obtenir le cache existant
    const cached = this.getCached(key);

    // Créer l'Observable réseau
    const network$ = from(fetchFn()).pipe(
      tap(data => {
        const cacheItem: CachedItem<T> = {
          data,
          timestamp: now,
          expiresAt: now + ttl
        };

        this.memoryCache.set(key, cacheItem);
        this.setToLocalStorage(key, cacheItem);
        console.log('Données réseau obtenues et mises en cache');
      }),
      catchError(err => {
        console.error('Erreur réseau:', err);
        // Retourner le cache en cas d'erreur réseau
        return cached ? of(cached.data) : throwError(() => err);
      }),
      shareReplay({ bufferSize: 1, refCount: false })
    );

    // Retourner d'abord le cache s'il existe
    if (cached) {
      console.log('Retour du cache puis mise à jour réseau');
      return merge(
        of(cached.data),  // Retourner immédiatement le cache
        network$          // Actualiser depuis le réseau
      );
    }

    // S'il n'y a pas de cache, récupérer uniquement depuis le réseau
    console.log('Pas de cache - Récupération réseau uniquement');
    return network$;
  }

  /**
   * Obtenir le cache (mémoire → localStorage)
   */
  private getCached(key: string): CachedItem<T> | null {
    // Vérifier le cache mémoire
    const memCached = this.memoryCache.get(key);
    if (memCached) return memCached;

    // Vérifier localStorage
    return this.getFromLocalStorage(key);
  }

  private getFromLocalStorage(key: string): CachedItem<T> | null {
    try {
      const item = localStorage.getItem(`offline:${key}`);
      if (!item) return null;
      return JSON.parse(item) as CachedItem<T>;
    } catch {
      return null;
    }
  }

  private setToLocalStorage(key: string, item: CachedItem<T>): void {
    try {
      localStorage.setItem(`offline:${key}`, JSON.stringify(item));
    } catch (error) {
      console.error('Erreur d\'écriture du localStorage:', error);
    }
  }
}

// Exemple d'utilisation
const offlineCache = new OfflineFirstCache<User[]>();

// Première visite - Récupération réseau
offlineCache.get('users', fetchUsers).subscribe({
  next: users => console.log('Utilisateurs:', users.length),
  error: err => console.error('Erreur:', err)
});

// Deuxième visite - Retourner d'abord le cache puis actualiser
// (Fonctionne même hors ligne car le cache existe)
offlineCache.get('users', fetchUsers).subscribe({
  next: users => console.log('Utilisateurs (offline-first):', users.length)
  // Sortie: "Retour du cache puis mise à jour réseau"
  // → Cache d'abord, puis mise à jour réseau si disponible
});
```

> [!IMPORTANT] Conception offline-first
> **Avantages :**
> - ✅ Fonctionne même hors ligne
> - ✅ Réponse instantanée (expérience rapide)
> - ✅ Mise à jour automatique quand le réseau est disponible
>
> **Points d'attention :**
> - ⚠️ Les utilisateurs peuvent voir des données obsolètes
> - ⚠️ Nécessité de visualiser l'état du réseau
> - ⚠️ Considération de la stratégie de synchronisation

## Gestion des erreurs de cache

### Problème : Gestion appropriée des erreurs de récupération du cache

Lors de la récupération du cache, des erreurs peuvent survenir (quota localStorage plein, corruption de données, erreur réseau, etc.). Une gestion appropriée des erreurs est nécessaire.

### Solution : Stratégie de récupération gracieuse

**Modèles de gestion des erreurs :**
1. **Erreur de lecture du cache** → Ignorer et récupérer depuis le réseau
2. **Erreur réseau** → Retourner le cache ancien si disponible
3. **Erreur d'écriture du cache** → Enregistrer et continuer (sans bloquer l'utilisateur)

```typescript
import { of, catchError, tap, retry, throwError } from 'rxjs';
class ResilientCache<T> {
  private memoryCache = new Map<string, CachedItem<T>>();
  private errorLog: Array<{ timestamp: Date; error: Error }> = [];

  /**
   * Obtenir des données avec gestion des erreurs
   */
  get(
    key: string,
    fetchFn: () => Observable<T>,
    ttl: number = 300000
  ): Observable<T> {
    return defer(() => {
      const now = Date.now();

      // Lire le cache en toute sécurité
      const cached = this.safeGetCache(key);
      if (cached && cached.expiresAt > now) {
        console.log('Hit du cache');
        return of(cached.data);
      }

      // Récupération réseau avec gestion des erreurs
      return fetchFn().pipe(
        retry({
          count: 3,
          delay: 1000
        }),
        tap(data => {
          // Écrire dans le cache en toute sécurité
          this.safeSetCache(key, {
            data,
            timestamp: now,
            expiresAt: now + ttl
          });
        }),
        catchError(err => {
          console.error('Erreur réseau:', err);

          // Retourner le cache ancien en cas d'erreur
          if (cached) {
            console.log('Utilisation du cache ancien en raison d\'une erreur réseau');
            return of(cached.data);
          }

          // Lancer une erreur s'il n'y a pas de cache
          return throwError(() => err);
        })
      );
    });
  }

  /**
   * Lire le cache en toute sécurité
   */
  private safeGetCache(key: string): CachedItem<T> | null {
    try {
      // Vérifier le cache mémoire
      const memCached = this.memoryCache.get(key);
      if (memCached) return memCached;

      // Vérifier localStorage
      const item = localStorage.getItem(`cache:${key}`);
      if (!item) return null;

      const parsed = JSON.parse(item) as CachedItem<T>;

      // Promouvoir vers le cache mémoire
      this.memoryCache.set(key, parsed);

      return parsed;
    } catch (error) {
      console.error('Erreur de lecture du cache:', error);
      this.logError(error as Error);
      return null; // Ignorer l'erreur et retourner null
    }
  }

  /**
   * Écrire dans le cache en toute sécurité
   */
  private safeSetCache(key: string, item: CachedItem<T>): void {
    try {
      // Écrire dans le cache mémoire
      this.memoryCache.set(key, item);

      // Écrire dans localStorage
      localStorage.setItem(`cache:${key}`, JSON.stringify(item));
    } catch (error) {
      console.error('Erreur d\'écriture du cache:', error);
      this.logError(error as Error);

      // Nettoyer le cache ancien si le quota est dépassé
      if (this.isQuotaExceeded(error as Error)) {
        console.log('Quota localStorage dépassé - Nettoyage du cache ancien');
        this.cleanOldCache();
      }
    }
  }

  /**
   * Vérifier si c'est une erreur de quota
   */
  private isQuotaExceeded(error: Error): boolean {
    return error.name === 'QuotaExceededError' ||
           error.message.includes('quota');
  }

  /**
   * Nettoyer le cache ancien
   */
  private cleanOldCache(): void {
    try {
      const keys: string[] = [];

      // Obtenir toutes les clés de cache
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key?.startsWith('cache:')) {
          keys.push(key);
        }
      }

      // Trier par date (plus ancien en premier)
      const items = keys
        .map(key => {
          try {
            const item = JSON.parse(localStorage.getItem(key)!);
            return { key, timestamp: item.timestamp };
          } catch {
            return { key, timestamp: 0 };
          }
        })
        .sort((a, b) => a.timestamp - b.timestamp);

      // Supprimer les 20% les plus anciens
      const removeCount = Math.ceil(items.length * 0.2);
      for (let i = 0; i < removeCount; i++) {
        localStorage.removeItem(items[i].key);
      }

      console.log(`Nettoyé ${removeCount} entrées de cache anciennes`);
    } catch (error) {
      console.error('Erreur de nettoyage du cache:', error);
    }
  }

  /**
   * Enregistrer les erreurs
   */
  private logError(error: Error): void {
    this.errorLog.push({
      timestamp: new Date(),
      error
    });

    // Garder seulement les 50 erreurs les plus récentes
    if (this.errorLog.length > 50) {
      this.errorLog = this.errorLog.slice(-50);
    }
  }

  /**
   * Obtenir le journal des erreurs
   */
  getErrorLog(): Array<{ timestamp: Date; error: Error }> {
    return [...this.errorLog];
  }
}
```

> [!WARNING] Gestion des erreurs de cache
> **Erreurs courantes :**
> - **QuotaExceededError** - Quota localStorage dépassé (généralement 5-10MB)
> - **Corruption de données** - JSON invalide, données corrompues
> - **Erreur réseau** - Timeout, erreur HTTP, pas de connexion
>
> **Stratégies de récupération :**
> - Ignorer les erreurs de lecture du cache et récupérer depuis le réseau
> - Retourner le cache ancien en cas d'erreur réseau
> - Nettoyer automatiquement le cache ancien quand le quota est dépassé
> - Enregistrer les erreurs pour l'analyse

## Stratégies de mise à jour

### Prefetch (préchargement)

**Prefetch :**
- Précharger les données susceptibles d'être nécessaires avant que l'utilisateur ne les demande
- Amélioration de l'expérience utilisateur (affichage instantané)
- Utilisation efficace du temps d'inactivité

**Scénarios de prefetch :**
- Précharger la page suivante lors du défilement
- Précharger les détails lors du survol d'un élément de liste
- Précharger les écrans suivants probables

```typescript
import { fromEvent, debounceTime, switchMap, tap } from 'rxjs';
class PrefetchCache<T> {
  private cache = new Map<string, CachedItem<T>>();

  /**
   * Prefetch - Précharger les données dans le cache
   */
  prefetch(key: string, fetchFn: () => Observable<T>, ttl: number = 300000): void {
    const now = Date.now();

    // Ne rien faire si déjà en cache
    if (this.cache.has(key)) {
      console.log(`Prefetch ignoré - Déjà en cache: ${key}`);
      return;
    }

    console.log(`Prefetch démarré: ${key}`);

    fetchFn().pipe(
      tap(data => {
        this.cache.set(key, {
          data,
          timestamp: now,
          expiresAt: now + ttl
        });
        console.log(`Prefetch terminé: ${key}`);
      }),
      catchError(err => {
        console.error(`Erreur de prefetch: ${key}`, err);
        return of(null); // Ignorer l'erreur
      })
    ).subscribe();
  }

  /**
   * Obtenir depuis le cache (généralement instantané grâce au prefetch)
   */
  get(key: string, fetchFn: () => Observable<T>, ttl: number = 300000): Observable<T> {
    const cached = this.cache.get(key);
    const now = Date.now();

    if (cached && cached.expiresAt > now) {
      console.log(`Hit du cache (prefetch): ${key}`);
      return of(cached.data);
    }

    console.log(`Miss du cache - Récupération: ${key}`);
    return fetchFn().pipe(
      tap(data => {
        this.cache.set(key, {
          data,
          timestamp: now,
          expiresAt: now + ttl
        });
      })
    );
  }
}

// Exemple d'utilisation : Prefetch lors du survol d'un élément de liste
const prefetchCache = new PrefetchCache<Post>();

// Créer un élément de liste dynamiquement
const listItem = document.createElement('div');
listItem.id = 'post-item-1';
listItem.textContent = 'Publication 1 (survolez pour prefetch)';
listItem.style.padding = '10px';
listItem.style.margin = '10px';
listItem.style.border = '1px solid #ccc';
listItem.style.cursor = 'pointer';
document.body.appendChild(listItem);

const postId = 1;

fromEvent(listItem, 'mouseenter').pipe(
  debounceTime(300) // Prefetch 300ms après le survol
).subscribe(() => {
  console.log('Survol détecté - Démarrage du prefetch');
  prefetchCache.prefetch(
    `post-${postId}`,
    () => from(
      fetch(`https://jsonplaceholder.typicode.com/posts/${postId}`)
        .then(r => r.json())
    )
  );
});

// Lors du clic, le cache est déjà prêt
fromEvent(listItem, 'click').subscribe(() => {
  console.log('Clic - Récupération de la publication');
  prefetchCache.get(
    `post-${postId}`,
    () => from(
      fetch(`https://jsonplaceholder.typicode.com/posts/${postId}`)
        .then(r => r.json())
    )
  ).subscribe(post => {
    console.log('Publication:', post.title); // Affichage instantané grâce au prefetch
  });
});
```

> [!TIP] Cas d'utilisation du prefetch
> **Scénarios efficaces :**
> - Défilement infini (précharger la page suivante)
> - Survol d'éléments de liste (précharger les détails)
> - Pagination (précharger la page suivante)
> - Navigation prédictive (précharger les écrans probables)
>
> **Points d'attention :**
> - ⚠️ Consommation de bande passante (éviter le prefetch excessif)
> - ⚠️ Utilisation de la mémoire (limiter le nombre de prefetch)
> - ⚠️ Prefetch uniquement des ressources nécessaires

## Code de test

Exemples de tests pour les stratégies de mise en cache.

```typescript
import { TestScheduler } from 'rxjs/testing';

describe('Stratégies de mise en cache', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('should cache data with shareReplay', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a-b-c-|', {
        a: { id: 1, name: 'Test' },
        b: { id: 2, name: 'Test2' },
        c: { id: 3, name: 'Test3' }
      });

      const cached$ = source$.pipe(
        shareReplay({ bufferSize: 1, refCount: false })
      );

      // Premier abonnement
      expectObservable(cached$).toBe('--a-b-c-|', {
        a: { id: 1, name: 'Test' },
        b: { id: 2, name: 'Test2' },
        c: { id: 3, name: 'Test3' }
      });

      // Deuxième abonnement (depuis le cache)
      expectObservable(cached$, '^-!').toBe('-c', {
        c: { id: 3, name: 'Test3' }
      });
    });
  });

  it('should invalidate cache after TTL', () => {
    testScheduler.run(({ cold, hot, expectObservable }) => {
      const cache = new TTLCache<any>();
      const ttl = 30; // 30 frames de temps virtuel

      const fetch$ = cold('a|', { a: { data: 'test' } });

      const result$ = cache.get('test-key', () => fetch$, ttl);

      // Première requête
      expectObservable(result$).toBe('a|', { a: { data: 'test' } });

      // Requête immédiate suivante (depuis le cache)
      expectObservable(result$, '---^').toBe('a|', { a: { data: 'test' } });

      // Après expiration du TTL (nouveau fetch)
      expectObservable(result$, '--------------------------------^').toBe('a|', {
        a: { data: 'test' }
      });
    });
  });
});
```

## Résumé

En maîtrisant les stratégies de mise en cache, vous pouvez améliorer considérablement les performances et l'expérience utilisateur de vos applications.

> [!IMPORTANT] Points importants
> - **shareReplay** : Mise en cache de base, éviter les appels API redondants
> - **TTL** : Définir la durée de vie du cache, empêcher les données obsolètes
> - **Invalidation** : Maintenir la cohérence des données lors de mutations
> - **Multi-niveaux** : Combiner mémoire et localStorage pour rapidité et persistance
> - **Offline-first** : Fournir un cache même hors ligne, améliorer l'expérience utilisateur
> - **Gestion des erreurs** : Récupération gracieuse, nettoyage automatique du cache

> [!TIP] Meilleures pratiques
> - **Conception TTL appropriée** : Équilibre entre fraîcheur des données et charge serveur
> - **Invalidation fine** : Invalidation ciblée avec système de tags
> - **Gestion de la mémoire** : Nettoyage automatique du cache ancien
> - **Visualisation de l'état** : Indiquer clairement le statut du cache à l'utilisateur
> - **Tests** : Tester les stratégies de cache avec TestScheduler

## Prochaines étapes

Après avoir maîtrisé les stratégies de mise en cache, passez aux modèles suivants :

- [Appels API](./api-calls.md) - Combiner appels API et stratégies de cache
- [Traitement de données temps réel](./real-time-data.md) - Mise en cache de données temps réel
- [Traitement de formulaires](./form-handling.md) - Mise en cache de données de formulaires
- [Traitement d'événements UI](./ui-events.md) - Mise en cache d'événements UI

## Sections connexes

- [Chapitre 2: Cold/Hot Observable](../observables/cold-and-hot-observables.md) - Fondements de shareReplay
- [Chapitre 4: Opérateurs](../operators/index.md) - Détails sur shareReplay, share, etc.
- [Chapitre 6: Gestion des erreurs](../error-handling/strategies.md) - Gestion des erreurs de cache

## Ressources de référence

- [RxJS officiel: shareReplay](https://rxjs.dev/api/operators/shareReplay) - Détails de shareReplay
- [RxJS officiel: share](https://rxjs.dev/api/operators/share) - Détails de share
- [MDN: localStorage](https://developer.mozilla.org/fr/docs/Web/API/Window/localStorage) - Utilisation de localStorage
- [MDN: IndexedDB](https://developer.mozilla.org/fr/docs/Web/API/IndexedDB_API) - Utilisation d'IndexedDB
