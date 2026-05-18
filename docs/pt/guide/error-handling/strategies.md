---
description: "Descreve uma estratégia abrangente de tratamento de erros para RxJS, incluindo como combinar os operadores catchError, retry, retryWhen e finalize, retry com backoff exponencial, classificação de erros e tratamento adequado, manipuladores de erros globais e outras estratégias robustas de tratamento de erros em TypeScript. Como implementar o tratamento robusto de erros no TypeScript."
---

# Estratégia de tratamento de erros do RxJS

O tratamento de erros no RxJS é um aspecto importante da programação reativa. A implementação do tratamento adequado de erros melhora a robustez e a confiabilidade do seu aplicativo. Este documento descreve as diferentes estratégias de tratamento de erros disponíveis no RxJS.

## Padrões básicos

O RxJS trata os erros como parte do ciclo de vida do Observable. O tratamento básico de erros inclui os seguintes métodos.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Erro.Observable
const error$ = throwError(() => new Error('Ocorreu um erro.')); // RxJS 7Formato de função recomendado a partir de agora

// Tratamento básico de erros
error$
  .pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Captura de erro:', message);
      return of('Valor de retorno após o erro');
    })
  )
  .subscribe({
    next: (value) => console.log('Valor:', value),
    error: (err) => console.error('Erros não tratados:', err),
    complete: () => console.log('Concluído'),
  });

// Saída:
// Captura de erro: Ocorreu um erro.
// Valor: Valor de retorno após o erro
// Concluído
```

## Várias estratégias de tratamento de erros

### 1. capturar erros e fornecer valores alternativos

Use o operador "catchError" para capturar erros e fornecer valores alternativos ou fluxos alternativos.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Erro de aquisição de dados'));

source$.pipe(
  catchError((error: unknown) => {
    const message = error instanceof Error ? error.message : String(error);
    console.error('Ocorreu um erro:', message);
    // Retornar dados alternativos
    return of({ isError: true, data: [], message: 'Exibir dados padrão' });
  })
).subscribe(data => console.log('Resultado:', data));

// Saída:
// Ocorreu um erro: Erro de aquisição de dados
// Resultado: {isError: true, data: Array(0), message: 'Exibir dados padrão'}
```

### Tente novamente se ocorrer um erro

Use o operador `retry` para tentar novamente o fluxo se ocorrer um erro (a partir da versão 7.3, o formato `retry({ count, delay })` é recomendado e substitui o antigo `retryWhen`).

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Erro #${attemptCount}`));
    }
    return of('Sucesso！');
  }),
  tap(() => console.log('Execução:', attemptCount)),
  retry(2), // Máx.2Novas tentativas
).subscribe({
  next: value => console.log('Valor:', value),
  error: err => console.error('Erro final:', err.message),
});

// Saída:
// Execução: 3
// Valor: Sucesso！
// Execução: 4
// Valor: Sucesso！
// Execução: 5
// ...
```

### 3. Repetir com backoff exponencial

O backoff exponencial, que aumenta gradualmente o intervalo de repetição, é eficaz, por exemplo, para solicitações de rede.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, tap, catchError } from 'rxjs';

function fetchWithRetry() {
  return throwError(() => new Error('Erro de rede')).pipe(
    // RxJS 7.3+ Recomendado: retry({ count, delay }) Formato
    retry({
      count: 5, // Máx.5Até três novas tentativas
      delay: (error: unknown, retryCount) => {
        const message = error instanceof Error ? error.message : String(error);
        console.log('Ocorreu um erro:', message);
        // Recuo exponencial (máx.10segundos)
        const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
        console.log(`${retryCount}Repetir uma segunda vez${delayMs}msExecutado após`);
        return timer(delayMs);
      }
    }),
    // Recuo final
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Todas as tentativas falharam:', message);
      return of({
        error: true,
        message: 'Falha na conexão. Por favor, tente novamente mais tarde.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Resultado:', result),
  error: (err) => console.error('Erros não tratados:', err),
});

// Saída:
// Ocorreu um erro: Erro de rede
// 1Repetir uma segunda vez2000msExecutado após
// Ocorreu um erro: Erro de rede
// 2Repetir uma segunda vez4000msExecutado após
// Ocorreu um erro: Erro de rede
// 3Repetir uma segunda vez8000msExecutado após
// Ocorreu um erro: Erro de rede
// 4Repetir uma segunda vez10000msExecutado após
// Ocorreu um erro: Erro de rede
// 5Repetir uma segunda vez10000msExecutado após
// Todas as tentativas falharam: Número máximo de tentativas excedido
// Resultado: {error: true, message: 'Falha na conexão. Por favor, tente novamente mais tarde.'}
```

### Liberação de recursos em caso de erro

Use o operador `finalize` para liberar recursos quando um fluxo finalizar **completo ou com erro**.
Finalize é útil quando você deseja garantir que o processo de limpeza seja realizado não apenas em caso de erro, mas também na conclusão normal.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Erro de processamento'))
  .pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Erro de processamento:', message);
      return throwError(() => error); // Repetir erro
    }),
    finalize(() => {
      isLoading = false;
      console.log('Redefinir status de carregamento:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valor:', value),
    error: (err) => console.error('Erro final:', err.message),
    complete: () => console.log('Concluído'),
  });

// Saída:
// Erro de processamento: Erro de processamento
// Erro final: Erro de processamento
// Redefinir status de carregamento: false
```

## Padrão de tratamento de erros

### Tratamento de erros, incluindo controle de exibição de elementos da interface do usuário

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Exibição do carregamento
  showLoadingIndicator();

  // Aquisição de dados (sucesso ou erro)
  return (
    shouldFail
      ? throwError(() => new Error('APIErro'))
      : of({ name: 'Dados', value: 42 })
  ).pipe(
    tap((data) => {
      // Com sucesso
      updateUI(data);
    }),
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      // Em caso de erroUIAtualizar
      showErrorMessage(message);
      // Retorna dados vazios ou valor padrão
      return of({ name: 'Padrão', value: 0 });
    }),
    finalize(() => {
      // Silenciar a tela de carregamento independentemente de sucesso ou erro
      hideLoadingIndicator();
    })
  );
}

// UIFunções auxiliares para operações
function showLoadingIndicator() {
  console.log('Exibição do carregamento');
}
function hideLoadingIndicator() {
  console.log('Ocultar carregamento');
}
function updateUI(data: { name: string; value: number }) {
  console.log('UIAtualizar:', data);
}
function showErrorMessage(message: any) {
  console.log('Exibição de erro:', message);
}

// Exemplo de uso
fetchData(true).subscribe();

// Saída:
// Exibição do carregamento
// Exibição de erro: APIErro
// Ocultar carregamento
```

### Tratamento de várias fontes de erro

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Simular váriasAPISimular solicitações
function getUser() {
  return of({ id: 1, name: 'Taro Yamada' });
}

function getPosts() {
  return throwError(() => new Error('Erro de aquisição de postagem'));
}

function getComments() {
  return throwError(() => new Error('Comentar erro de aquisição'));
}

// Recuperar todos os dados e permitir erros parciais
forkJoin({
  user: getUser().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Erro de aquisição do usuário:', message);
      return of(null); // No caso de um erronullRetorna uma matriz vazia
    })
  ),
  posts: getPosts().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Erro de aquisição de postagem:', message);
      return of([]); // A matriz vazia é retornada em caso de erro
    })
  ),
  comments: getComments().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Comentar erro de aquisição:', message);
      return of([]); // A matriz vazia é retornada em caso de erro
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Adiciona um sinalizador para indicar se houve um erro parcial
      hasErrors:
        !result.user ||
        result.posts.length === 0 ||
        result.comments.length === 0,
    }))
  )
  .subscribe((data) => {
    console.log('Resultado final:', data);

    if (data.hasErrors) {
      console.log(
        'Falha na recuperação de alguns dados, mas mostra os dados disponíveis'
      );
    }
  });

// Saída:
// Erro de aquisição de postagem: Erro de aquisição de postagem
// Comentar erro de aquisição: Comentar erro de aquisição
// Resultado final: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// Falha na recuperação de alguns dados, mas mostra os dados disponíveis
```

## Melhores práticas para o tratamento de erros

1. **Sempre capture erros**: sempre adicione o tratamento de erros na cadeia Observable. Isso é especialmente importante para fluxos de longa duração.

2.**Forneça mensagens de erro significativas**: inclua informações no objeto de erro para ajudar a determinar onde ele ocorreu e o que o causou.

3, **liberar recursos adequadamente**: use `finalize` para garantir que os recursos sejam liberados independentemente de sucesso ou falha.

4, **Considere estratégias de repetição**: especialmente para operações de rede, a implementação de uma estratégia de repetição adequada aumenta a confiabilidade.

5. **Tratamento de erros amigável ao usuário**: na interface do usuário, forneça informações que os usuários possam entender, em vez de exibir mensagens de erro técnicas como elas são.

```ts
// Exemplo.：Conversão para mensagens de erro fáceis de usar
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'A sessão expirou. Faça login novamente.';
  } else if (error.status === 404) {
    return 'O recurso solicitado não pôde ser encontrado.';
  } else if (error.status >= 500) {
    return 'Ocorreu um erro no servidor. Tente novamente mais tarde.';
  }
  return 'Ocorreu um erro inesperado.';
}
```

## Resumo.

O tratamento de erros no RxJS é uma parte importante para garantir a robustez do aplicativo. Usando a combinação certa de operadores como `catchError`, `retry` e `finalize`, é possível lidar com uma variedade de cenários de erro. Projete uma estratégia abrangente de tratamento de erros para melhorar a experiência do usuário, em vez de simplesmente capturar os erros.

## 🔗 Seções relacionadas.

- **[Erros comuns e como lidar com eles](/pt/guide/anti-patterns/common-mistakes#9-error-grasping)** - Reveja os antipadrões de tratamento de erros.
- **[retry and catchError](/pt/guide/error-handling/retry-catch)** - Explica a utilização mais detalhada